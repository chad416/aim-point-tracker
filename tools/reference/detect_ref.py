"""NumPy / SciPy reference for aimtrk detect and centroid (SPEC §6.1, §6.2, §9.2).

Written from the SPEC, not translated from the C: connected components come
from scipy.ndimage.label (8-connected), the border from binary_dilation, and
the rim fit from numpy.linalg.lstsq. Only the fixed-point background (DN * 256)
and the float32 values the C keeps are copied on purpose, so the two can be
compared to 1e-4 px.
"""
from __future__ import annotations

import math

import numpy as np
from scipy import ndimage

CELL = 4
MAD_TO_SIGMA = 1.4826
SAT, RIM, RIM_FAILED, EDGE, ON_CHANGE = 0x01, 0x02, 0x04, 0x08, 0x10
RECOVER_R, RECOVER_PEAKS, RECOVER_CONTRAST, RECOVER_MAX_SIG = 6, 4, 2.0, 2.5
REJECT_NAMES = ("small", "large", "weak", "elongated")

DEFAULT_CFG = dict(bg_shift=4, k_sigma=6.0, t_min=8.0, min_area=3, max_area=300, max_extent=24,
                   max_elong=3.0, min_snr=8.0, sat_level=255)

f32 = np.float32


def hist_median(h: np.ndarray, lo: float, n: int) -> float:
    """Median of integer samples in unit bins starting at lo, linear inside the bin."""
    target = 0.5 * n
    cum = np.concatenate([[0], np.cumsum(h)])
    i = int(np.argmax((cum[1:] >= target) & (h > 0)))
    return lo + i + (target - cum[i]) / h[i]


def noise_sigma(img: np.ndarray, bg256: np.ndarray) -> float:
    """MAD noise estimate on every 4th pixel of every 4th row (offset 2), as float32."""
    b = (bg256 + 128) >> 8
    d = img[2::CELL, 2::CELL].astype(np.int64) - b
    n = d.size
    h = np.bincount((d + 255).ravel(), minlength=511)
    med = hist_median(h, -255.5, n)
    m = math.floor(med + 0.5)
    a = np.abs(d - m).ravel()
    ha = np.bincount(a, minlength=1)
    # bin 0 covers 0..0.5, bin k covers k - 0.5..k + 0.5
    target = 0.5 * n
    cum = np.concatenate([[0], np.cumsum(ha)])
    k = int(np.argmax((cum[1:] >= target) & (ha > 0)))
    frac = (target - cum[k]) / ha[k]
    mad = 0.5 * frac if k == 0 else (k - 0.5) + frac
    return float(f32(MAD_TO_SIGMA * mad))


def rim_fit(win: np.ndarray, bgw: np.ndarray, mask: np.ndarray, sat_level: int, min_signal: float):
    """Fit ln(s) = a + b u + c v + d u^2 + e uv + f v^2, weights s^2; return the peak (u, v) or None."""
    h, w = win.shape
    s = win.astype(np.float64) - bgw.astype(np.float64)
    use = mask & (win < sat_level) & (s > min_signal)
    if use.sum() < 8:
        return None
    vv, uu = np.nonzero(use)
    u = uu - 0.5 * (w - 1)
    v = vv - 0.5 * (h - 1)
    sv = s[use]
    A = np.column_stack([np.ones_like(u), u, v, u * u, u * v, v * v]) * sv[:, None]  # sqrt(weight) = s
    p, *_ = np.linalg.lstsq(A, np.log(sv) * sv, rcond=None)
    _, b, c, d, e, f = p
    det = 4 * d * f - e * e
    if d >= 0 or f >= 0 or det <= 0:
        return None
    u0 = (-2 * f * b + c * e) / det
    v0 = (-2 * d * c + e * b) / det
    if abs(u0) > 0.5 * (w - 1) + 1 or abs(v0) > 0.5 * (h - 1) + 1:
        return None
    return u0 + 0.5 * (w - 1), v0 + 0.5 * (h - 1)


def centroid(win: np.ndarray, bgw: np.ndarray, mask: np.ndarray, sigma: float, sat_level: int,
             x0: int = 0, y0: int = 0) -> dict | None:
    """at_centroid() on a window (uint8), its background (float32 DN) and mask (bool)."""
    s = win.astype(np.float64) - bgw.astype(np.float64)
    sm = np.where(mask, s, 0.0)
    wgt = np.clip(sm, 0.0, None)
    s0 = wgt.sum()
    if s0 <= 0:
        return None
    vv, uu = np.mgrid[0:win.shape[0], 0:win.shape[1]]
    mu, mv = (wgt * uu).sum() / s0, (wgt * vv).sum() / s0
    cuu = (wgt * uu * uu).sum() / s0 - mu * mu
    cvv = (wgt * vv * vv).sum() / s0 - mv * mv
    cuv = (wgt * uu * vv).sum() / s0 - mu * mv
    ev = np.linalg.eigvalsh(np.array([[cuu, cuv], [cuv, cvv]]))
    sig_n = max(float(sigma), 0.5)
    n_sat = int((mask & (win >= sat_level)).sum())
    peak = max(float(sm[mask].max()), 0.0)
    spot = dict(x=x0 + mu, y=y0 + mv, cx=x0 + mu, cy=y0 + mv, sum=s0, peak=peak,
                snr=float(f32(peak / sig_n)), sig_major=float(f32(math.sqrt(max(ev[1], 0.0)))),
                sig_minor=float(f32(math.sqrt(max(ev[0], 0.0)))), n_sat=n_sat, flags=0)
    if n_sat:
        spot["flags"] |= SAT
        r = rim_fit(win, bgw, mask, sat_level, float(f32(4.0 * sig_n)))
        if r is None:
            spot["flags"] |= RIM_FAILED
        else:
            spot["x"], spot["y"] = x0 + r[0], y0 + r[1]
            spot["flags"] |= RIM
    return spot


class DetectRef:
    """Same state and steps as at_detect, in NumPy."""

    def __init__(self, width: int, height: int, **cfg):
        self.w, self.h = width, height
        self.cfg = {**DEFAULT_CFG, **cfg}
        self.bg256: np.ndarray | None = None
        self.stats: dict = {}

    def cell_sums(self, img: np.ndarray) -> np.ndarray:
        return img.astype(np.int64).reshape(self.h // CELL, CELL, self.w // CELL, CELL).sum(axis=(1, 3))

    def learn(self, img: np.ndarray) -> None:
        self.bg256 = self.cell_sums(img) * 16

    def _recover(self, img: np.ndarray, blob: np.ndarray, bgw_full: np.ndarray, delta256: int,
                 sigma: float) -> list[dict]:
        """Step 6 (SPEC 6.1, M4): spots inside a blob rejected as too large, each measured against
        the brightest pixel on the border of its window."""
        c = self.cfg
        ys, xs = np.nonzero(blob)  # raster order, as the C run scan
        ex = img[ys, xs].astype(np.float64) - bgw_full[ys, xs].astype(np.float64)
        delta = delta256 / 256.0
        sig_n = max(float(sigma), 0.5)
        peaks, out = [], []
        while len(peaks) < RECOVER_PEAKS:
            ok = ex > 0.0
            for (qx, qy) in peaks:
                ok &= ~((np.abs(xs - qx) <= RECOVER_R) & (np.abs(ys - qy) <= RECOVER_R))
            if not ok.any():
                break
            i = int(np.argmax(np.where(ok, ex, -np.inf)))
            best = ex[i]
            if best <= RECOVER_CONTRAST * delta:
                break
            bx, by = int(xs[i]), int(ys[i])
            peaks.append((bx, by))
            x0, y0 = max(bx - RECOVER_R, 0), max(by - RECOVER_R, 0)
            x1, y1 = min(bx + RECOVER_R, self.w - 1), min(by + RECOVER_R, self.h - 1)
            win = img[y0:y1 + 1, x0:x1 + 1]
            exw = win.astype(np.float64) - bgw_full[y0:y1 + 1, x0:x1 + 1].astype(np.float64)
            border = np.ones(win.shape, dtype=bool)
            border[1:-1, 1:-1] = False
            ring = float(exw[border].max())
            contrast = best - ring
            if contrast <= RECOVER_CONTRAST * delta or contrast / sig_n < float(f32(c["min_snr"])):
                continue
            lvl = (bgw_full[y0:y1 + 1, x0:x1 + 1] + f32(ring)).astype(np.float32)
            core = (win.astype(np.float64) - lvl.astype(np.float64)) > delta
            area = int(core.sum())
            if area < c["min_area"]:
                continue
            mask = ndimage.binary_dilation(core, structure=np.ones((3, 3), dtype=bool))
            s = centroid(win, lvl, mask, sigma, c["sat_level"], x0, y0)
            bad = s is None or f32(s["snr"]) < f32(c["min_snr"]) or s["sig_minor"] <= 0
            bad = bad or f32(s["sig_major"]) / f32(s["sig_minor"]) > f32(c["max_elong"])
            if bad or f32(s["sig_major"]) > f32(RECOVER_MAX_SIG):
                continue
            s["area"] = area
            s["flags"] |= ON_CHANGE
            if x0 == 0 or y0 == 0 or x1 == self.w - 1 or y1 == self.h - 1:
                s["flags"] |= EDGE
            out.append(s)
        return out

    def _stands_out(self, img: np.ndarray, bgw_full: np.ndarray, s: dict, sigma: float) -> bool:
        """M4: the peak against the brightest border pixel of its window (C stands_out())."""
        cx, cy = math.floor(float(f32(s["x"])) + 0.5), math.floor(float(f32(s["y"])) + 0.5)
        x0, y0 = max(cx - RECOVER_R, 0), max(cy - RECOVER_R, 0)
        x1, y1 = min(cx + RECOVER_R, self.w - 1), min(cy + RECOVER_R, self.h - 1)
        exw = img[y0:y1 + 1, x0:x1 + 1].astype(np.float64) - bgw_full[y0:y1 + 1, x0:x1 + 1].astype(np.float64)
        yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
        ring_px = (np.abs(yy - cy) == RECOVER_R) | (np.abs(xx - cx) == RECOVER_R)  # a cut side has none
        ring = max(float(exw[ring_px].max()), 0.0) if ring_px.any() else 0.0
        peak = float(f32(s["peak"]))
        return ring <= 0.5 * peak or peak - ring >= float(f32(self.cfg["min_snr"])) * max(float(sigma), 0.5)

    def frame(self, img: np.ndarray) -> list[dict]:
        c = self.cfg
        if self.bg256 is None:
            self.learn(img)
            self.stats = {"learning": True}
            return []
        sigma = noise_sigma(img, self.bg256)
        delta = max(float(c["k_sigma"]) * sigma, float(c["t_min"]))
        delta256 = math.floor(delta * 256.0 + 0.5)
        # Highest background of the cell and its 8 neighbours (sharp edges, SPEC 6.1).
        hi = ndimage.maximum_filter(self.bg256, size=3, mode="constant", cval=0)
        thr = np.minimum((hi + delta256) >> 8, 255)
        thr_px = np.repeat(np.repeat(thr, CELL, axis=0), CELL, axis=1)
        labels, n = ndimage.label(img > thr_px, structure=np.ones((3, 3), dtype=int))
        bgw_full = np.repeat(np.repeat((self.bg256 / 256.0).astype(np.float32), CELL, 0), CELL, 1)
        # Blob order: raster order of the first pixel, as the C run scan.
        first = ndimage.minimum(np.arange(labels.size).reshape(labels.shape), labels, np.arange(1, n + 1))
        order = np.argsort(first) + 1 if n else []
        objects = ndimage.find_objects(labels)
        spots, rej, recovered = [], dict.fromkeys(REJECT_NAMES, 0), 0
        freeze = np.zeros_like(self.bg256, dtype=bool)
        for lab in order:
            sl = objects[lab - 1]
            ys, xs = sl
            area = int((labels[sl] == lab).sum())
            ext_w, ext_h = xs.stop - xs.start, ys.stop - ys.start
            if area < c["min_area"]:
                rej["small"] += 1
                continue
            if area > c["max_area"] or ext_w > c["max_extent"] or ext_h > c["max_extent"]:
                rej["large"] += 1
                found = self._recover(img, labels == lab, bgw_full, delta256, sigma)
                spots += found
                recovered += len(found)
                continue
            wy0, wx0 = max(ys.start - 1, 0), max(xs.start - 1, 0)
            wy1, wx1 = min(ys.stop, self.h - 1), min(xs.stop, self.w - 1)
            win_sl = (slice(wy0, wy1 + 1), slice(wx0, wx1 + 1))
            mask = ndimage.binary_dilation(labels[win_sl] == lab, structure=np.ones((3, 3), dtype=bool))
            s = centroid(img[win_sl], bgw_full[win_sl], mask, sigma, c["sat_level"], wx0, wy0)
            if s is None or f32(s["snr"]) < f32(c["min_snr"]):
                rej["weak"] += 1
                continue
            if s["sig_minor"] <= 0 or f32(s["sig_major"]) / f32(s["sig_minor"]) > f32(c["max_elong"]):
                rej["elongated"] += 1
                continue
            if not self._stands_out(img, bgw_full, s, sigma):
                rej["weak"] += 1
                continue
            cy0, cy1 = ys.start // CELL - 1, (ys.stop - 1) // CELL + 1
            cx0, cx1 = xs.start // CELL - 1, (xs.stop - 1) // CELL + 1
            freeze[max(cy0, 0):cy1 + 1, max(cx0, 0):cx1 + 1] = True
            s["area"] = area
            if xs.start == 0 or ys.start == 0 or xs.stop >= self.w or ys.stop >= self.h:
                s["flags"] |= EDGE
            spots.append(s)
        mean = self.cell_sums(img) * 16
        diff = mean - self.bg256
        step = np.sign(diff) * (np.abs(diff) >> c["bg_shift"])  # truncation toward zero, as C
        snap = (np.abs(diff) > delta256 // 2) & ~freeze  # a scene change: take the new mean at once
        self.bg256 = np.where(freeze, self.bg256, np.where(snap, mean, self.bg256 + step))
        self.stats = {"sigma": sigma, "blobs": int(n) - rej["small"], "spots": len(spots), "rejected": rej,
                      "frozen_cells": int(freeze.sum()), "snapped_cells": int(snap.sum()), "learning": False,
                      "recovered": recovered,
                      "bg_mean": float(self.bg256.mean() / 256.0)}
        return spots
