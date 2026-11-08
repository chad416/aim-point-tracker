"""Lane scenarios for the tracker (SPEC §6.5, §8, §9.3).

A scenario is one or more weapons on the lane: each with an aim slot, an aim
point that moves over lane time, trigger breaks, an emitter misalignment, and
times when it is lowered or its emitter is blocked. generate() turns it into
what aim_host sees:

  frames.bin   camera frames (region renderer, below), stamped on the
               tracker's own clock (Clock: offset and drift against lane time)
  can_in.log   the lane's CAN traffic as the tracker receives it: SYNC every
               10 ms (with reception jitter), and per shot the weapon's SHOT_EVENT and
               SHOT_DETAIL; candump -L format
  truth.json   every emitter pulse and every shot, with true screen positions

The weapons follow the weapon firmware v0.6 (SPEC §6.5): an aim pulse in the middle of each
exposure of the weapon's slot; at a break, the shot pulse in the first
exposure whose pulse starts at least 2 ms later, replacing the weapon's aim
pulse in that frame. Each pulse is placed on the weapon's own estimate of lane
time (Gaussian error, sync_sigma_us).

Region renderer: full frames take 26-48 ms in NumPy (SPEC §12), so a set of
noisy background frames (with the interference) is rendered once, and each
frame is one of them with the spots rendered over it in small patches with
their own noise. The background noise therefore repeats with the cache
period (48 frames by default); the spots' noise does not.
"""
from __future__ import annotations

import dataclasses
import json
import math
import pathlib
from typing import Callable

import cantools
import numpy as np

import camsim

ROOT = pathlib.Path(__file__).resolve().parents[1]
DBC = cantools.database.load_file(str(ROOT / "protocol" / "tracker.dbc"))

FRAME_US = 10_000
EXPOSURE_START_US = 4_500
PULSE_START_US = 4_750
PULSE_US = 500
SAT_MARGIN_PX = 3  # a spot (sigma about 1.2 px) this close to saturated background is not visible
MARGIN_US = 2_000  # aimtrk/match.h AT_STROBE_MARGIN_US
SYNC_PERIOD_US = 10_000


def shot_frame(t_break_us: int) -> int:
    """First exposure whose pulse starts >= 2 ms after the break (aimtrk/match.h)."""
    return -((-(t_break_us + MARGIN_US - PULSE_START_US)) // FRAME_US)


def nominal_total_dn(peak_dn: float = 150.0, sigma_px: float = 1.2) -> float:
    return peak_dn * 2 * math.pi * sigma_px**2


@dataclasses.dataclass
class Weapon:
    node: int
    slot: int
    aim: Callable[[float], tuple[float, float]]  # lane time (s) -> bore aim point (mm)
    breaks_s: list[float] = dataclasses.field(default_factory=list)
    mis_mrad: tuple[float, float] = (0.0, 0.0)
    raised: Callable[[float], bool] = lambda t: True     # aim pulses only while raised
    blocked: Callable[[float], bool] = lambda t: False   # emitter covered: no light
    total_dn: float = dataclasses.field(default_factory=nominal_total_dn)
    brightness: Callable[[float], float] = lambda t: 1.0  # emitter energy factor over time
    sync_sigma_us: float = 20.0
    muzzle_mm: tuple[float, float, float] = (0.0, 0.0, 5000.0)


@dataclasses.dataclass
class Clock:
    """The tracker's clock against lane time."""
    local0_us: int = 7_000_000_000
    lane0_us: int = 1_000_000_000
    drift_ppm: float = 40.0

    def local(self, lane_us: float) -> int:
        return int(round(self.local0_us + (lane_us - self.lane0_us) * (1.0 + self.drift_ppm * 1e-6)))


@dataclasses.dataclass
class Interference:
    """SPEC §8 interference set, in image pixels. Periodic over `period` frames."""
    leak_dn: float = 0.0          # projector leak: peak of the scenario image at 850 nm, DN
    window: bool = False          # a sunlit window at the top right of the view
    glints: bool = False          # specular glints from a target frame
    passer: tuple[int, int] | None = None  # exposures [a, b) during which a person crosses the view
    extra_hot: float = 0.0        # extra hot pixels (fraction)
    period: int = 48
    seed: int = 77

    def __post_init__(self):
        self._pattern = None

    def pattern(self, h: int, w: int) -> np.ndarray:
        """Static part of the projected scene (smooth blobs, 0..1)."""
        if self._pattern is None:
            rng = np.random.default_rng(self.seed)
            yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
            p = np.zeros((h, w), np.float32)
            for _ in range(6):
                cx, cy = rng.uniform(0, w), rng.uniform(0, h)
                s = rng.uniform(60, 200)
                p += rng.uniform(0.3, 1.0) * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * s * s))
            self._pattern = (0.6 * p / p.max()).astype(np.float32)
        return self._pattern

    def signal(self, j: int, h: int, w: int, rows=slice(None), cols=slice(None)) -> np.ndarray | float:
        """Extra signal (DN) of cache frame j over the given rows and columns."""
        r = np.arange(h)[rows][:, None]
        c = np.arange(w)[cols][None, :]
        out = np.zeros((r.shape[0], c.shape[1]), np.float32)
        if self.leak_dn > 0:
            flicker = 1.0 + 0.1 * math.sin(2 * math.pi * 0.37 * j)
            out += self.leak_dn * flicker * self.pattern(h, w)[rows, cols]
            # A target silhouette moving across the scene (one crossing per period).
            x0 = 100 + (w - 300) * (j % self.period) / self.period
            body = ((c >= x0) & (c < x0 + 90) & (r >= 330) & (r < 560)).astype(np.float32)
            out += self.leak_dn * flicker * 0.9 * body
        if self.window:
            out += 300.0 * ((c >= 1080) & (c < 1240) & (r >= 30) & (r < 150))
        if self.glints:
            for (gr, gc) in ((220, 300), (240, 900), (600, 700)):
                out += 70.0 * ((r >= gr) & (r < gr + 2) & (c >= gc) & (c < gc + 14))
        return out

    def occluded(self, k: int, w: int) -> tuple[int, int, int, int] | None:
        """Image rectangle (r0, r1, c0, c1) hidden by the passer-by in exposure k."""
        if self.passer is None or not (self.passer[0] <= k < self.passer[1]):
            return None
        frac = (k - self.passer[0]) / max(1, self.passer[1] - self.passer[0])
        c0 = int(-260 + (w + 260) * frac)
        return 250, 800, max(c0, 0), min(c0 + 260, w)


class RegionRenderer:
    def __init__(self, cam: camsim.Camera, rng: np.random.Generator, interference: Interference | None = None):
        self.cam, self.rng = cam, rng
        self.interf = interference or Interference()
        h, w = cam.lane.height, cam.lane.width
        if self.interf.extra_hot > 0:
            n = int(self.interf.extra_hot * h * w)
            fp = np.random.default_rng(self.interf.seed + 1)
            cam.hot_idx = np.concatenate([cam.hot_idx, fp.choice(h * w, n, replace=False)])
            cam.hot_val = np.concatenate([cam.hot_val, fp.uniform(40, 255, n).astype(np.float32)])
        self.cache = []
        for j in range(self.interf.period):
            sig = np.array(cam.ambient, dtype=np.float32) + self.interf.signal(j, h, w)
            self.cache.append(cam.expose(sig, rng))

    def frame(self, k: int, spots: list[camsim.Spot]) -> tuple[np.ndarray, list[dict]]:
        cam, interf = self.cam, self.interf
        h, w = cam.lane.height, cam.lane.width
        j = k % interf.period
        img = self.cache[j].copy()
        exposure = (k * FRAME_US + EXPOSURE_START_US, k * FRAME_US + EXPOSURE_START_US + 1000)
        occ = interf.occluded(k, w)
        truth = []
        for s in spots:
            probe = cam.add_spot(np.zeros((1, 1), np.float32), dataclasses.replace(s, total_dn=0.0), exposure)
            u, v = probe["u"], probe["v"]
            hidden = occ is not None and occ[0] <= v < occ[1] and occ[2] <= u < occ[3]
            # On a saturated background (the sunlit window) a spot cannot be seen at all, nor one that
            # straddles its edge: saturated pixels within SAT_MARGIN_PX (2.5 spot sigmas) of the centre.
            iu, iv = int(round(u)), int(round(v))
            if 0 <= iu < w and 0 <= iv < h:
                rs = slice(max(iv - SAT_MARGIN_PX, 0), min(iv + SAT_MARGIN_PX + 1, h))
                cs = slice(max(iu - SAT_MARGIN_PX, 0), min(iu + SAT_MARGIN_PX + 1, w))
                level = np.asarray(cam.ambient[rs, cs], np.float32) + interf.signal(j, h, w, rs, cs)
                hidden = hidden or float(level.max()) + cam.sensor.black_dn >= 250.0
            r0, r1 = max(int(v) - 12, 0), min(int(v) + 13, h)
            c0, c1 = max(int(u) - 12, 0), min(int(u) + 13, w)
            if hidden or r1 <= r0 or c1 <= c0:
                rec = dict(probe, total_dn=s.total_dn, rendered=False, hidden=hidden)
                truth.append(rec)
                continue
            rows, cols = slice(r0, r1), slice(c0, c1)
            sig = np.array(cam.ambient[rows, cols], dtype=np.float32) + interf.signal(j, h, w, rows, cols)
            # Overlapping spots: render into the patch on top of what is already there.
            rec = cam.add_spot(sig, s, exposure, origin=(r0, c0))
            for other in truth:
                if other.get("rendered") and abs(other["u"] - u) < 25 and abs(other["v"] - v) < 25:
                    o = dataclasses.replace(other["_spot"])
                    cam.add_spot(sig, o, exposure, origin=(r0, c0))
            img[rows, cols] = cam.expose(sig, self.rng, rows, cols)
            rec.update(rendered=True, hidden=False, _spot=s)
            truth.append(rec)
        if occ is not None:
            r0, r1, c0, c1 = occ
            if c1 > c0:
                dark = cam.sensor.black_dn + self.rng.normal(0, 0.5, (r1 - r0, c1 - c0))
                img[r0:r1, c0:c1] = np.clip(np.floor(dark + 0.5), 0, 255).astype(np.uint8)
        for rec in truth:
            rec.pop("_spot", None)
        return img, truth


@dataclasses.dataclass
class Scenario:
    weapons: list[Weapon]
    n_slots: int = 2
    t0_s: float = 1000.0          # lane time of the first frame
    duration_s: float = 2.0
    drop_k: set[int] = dataclasses.field(default_factory=set)  # exposures the camera loses
    sync_jitter_us: float = 20.0  # SYNC reception delay (one-sided)
    camera_jitter_us: float = 5.0  # frame timestamp jitter
    shot_event_delay_us: int = 1200  # break -> SHOT_EVENT received (firmware T03: <= 2 ms)
    clock: Clock = dataclasses.field(default_factory=Clock)
    interference: Interference | None = None
    seed: int = 1


def _frame(name: str, node: int, **signals) -> tuple[int, bytes]:
    msg = DBC.get_message_by_name(name)
    return msg.frame_id + (node - 1 if node > 0 else 0), msg.encode(signals, strict=True)


def _aim_point(w: Weapon, t_s: float) -> np.ndarray:
    return np.asarray(w.aim(t_s), dtype=float)


def generate(scn: Scenario, out_dir: pathlib.Path, cam: camsim.Camera | None = None,
             frames_opener: Callable | None = None) -> dict:
    """Write can_in.log, the frames and truth.json for the scenario. Returns the truth.

    The frames go to out_dir/frames.bin, or, if frames_opener is given, to the
    binary file object it returns once can_in.log is complete (e.g. the stdin of
    an aim_host process reading --frames -); it is closed at the end."""
    out_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(scn.seed)
    cam = cam or camsim.Camera()
    rend = RegionRenderer(cam, rng, scn.interference)
    clk = scn.clock
    t0_us = int(round(scn.t0_s * 1e6))
    k_first = -((-(t0_us - EXPOSURE_START_US)) // FRAME_US)
    k_last = k_first + int(round(scn.duration_s * 1e6 / FRAME_US))

    # Shots: frame, number, CAN frames.
    can = []
    shots = []
    shot_at = {}  # (node, k) -> shot record
    for w in scn.weapons:
        for n, b in enumerate(sorted(w.breaks_s), start=1):
            tb = int(round(b * 1e6))
            # The weapon stamps the break and picks the shot frame on its own lane-time estimate.
            t_weapon = int(round(tb + rng.normal(0, w.sync_sigma_us)))
            k = shot_frame(t_weapon)
            detail = {"takeup_ms": 4 * int(rng.integers(20, 80)), "squeeze_ms": 4 * int(rng.integers(40, 150)),
                      "jerk": int(rng.integers(5, 40)), "muzzle_rate": int(rng.integers(2, 20)),
                      "cant": int(rng.integers(-5, 6)), "flinch": round(float(rng.uniform(-1.5, 1.5)), 1)}
            rec = {"node": w.node, "shot_no": n, "t_break_us": tb, "t_weapon_us": t_weapon, "k": k,
                   "detail": detail}
            shots.append(rec)
            shot_at[(w.node, k)] = rec
            t_stamp = t_weapon % (1 << 32)
            fid, data = _frame("SHOT_EVENT_N1", w.node, shot_no=n, t_us=t_stamp, mode=1, chambered=1,
                               rounds_left=max(0, 30 - n))
            can.append((clk.local(tb + scn.shot_event_delay_us), fid, data))
            fid, data = _frame("SHOT_DETAIL_N1", w.node, shot_no=n, **detail)
            can.append((clk.local(tb + scn.shot_event_delay_us + 300), fid, data))

    # SYNC from 0.4 s before the first frame to the end.
    t = (t0_us - 400_000) // SYNC_PERIOD_US * SYNC_PERIOD_US
    while t < k_last * FRAME_US:
        fid, data = _frame("SYNC", 0, isotp_pci=4, lane_time_us=t % (1 << 32))
        can.append((clk.local(t) + int(abs(rng.normal(0, scn.sync_jitter_us))), fid, data))
        t += SYNC_PERIOD_US
    can.sort(key=lambda c: c[0])
    with open(out_dir / "can_in.log", "w") as f:
        for tl, fid, data in can:
            f.write(f"({tl // 1_000_000}.{tl % 1_000_000:06d}) can0 {fid:03X}#{data.hex().upper()}\n")

    # Frames.
    pulses = []
    with (frames_opener() if frames_opener else open(out_dir / "frames.bin", "wb")) as f:
        for index, k in enumerate(range(k_first, k_last)):
            spots, meta = [], []
            for w in scn.weapons:
                shot = shot_at.get((w.node, k))
                start = k * FRAME_US + PULSE_START_US + rng.normal(0, w.sync_sigma_us)
                t_mid_s = (start + PULSE_US / 2) * 1e-6
                if shot is None and not (k % scn.n_slots == w.slot and w.raised(t_mid_s)):
                    continue
                p0 = camsim.emitter_point(_aim_point(w, start * 1e-6), w.mis_mrad, w.muzzle_mm)
                p1 = camsim.emitter_point(_aim_point(w, (start + PULSE_US) * 1e-6), w.mis_mrad, w.muzzle_mm)
                vel = (p1 - p0) / (PULSE_US * 1e-6)
                kind = "shot" if shot else "aim"
                m = {"node": w.node, "k": k, "kind": kind, "shot_no": shot["shot_no"] if shot else None,
                     "t_mid_us": start + PULSE_US / 2, "blocked": bool(w.blocked(t_mid_s))}
                if m["blocked"]:
                    mid = 0.5 * (p0 + p1)
                    m.update(x_mm=float(mid[0]), y_mm=float(mid[1]), rendered=False)
                    pulses.append(m)
                    continue
                spots.append(camsim.Spot(xy_mm=(float(p0[0]), float(p0[1])), total_dn=w.total_dn * w.brightness(t_mid_s),
                                         t_us=(start, start + PULSE_US), vel_mm_s=(float(vel[0]), float(vel[1])),
                                         weapon=w.node, kind=kind))
                meta.append(m)
            img, recs = rend.frame(k, spots)
            for m, r in zip(meta, recs):
                m.update(x_mm=r["x_mm"], y_mm=r["y_mm"], u=r["u"], v=r["v"], rendered=r["rendered"],
                         hidden=r.get("hidden", False))
                pulses.append(m)
            if k in scn.drop_k:
                continue
            t_local = clk.local(k * FRAME_US + EXPOSURE_START_US) + int(round(rng.normal(0, scn.camera_jitter_us)))
            camsim.write_frame(f, img, index, t_local)
    for s in shots:
        p = [q for q in pulses if q["node"] == s["node"] and q["k"] == s["k"] and q["kind"] == "shot"]
        s.update(p[0] if p else {"rendered": False})
        s["dropped"] = s["k"] in scn.drop_k
    truth = {"k_first": k_first, "k_last": k_last, "n_slots": scn.n_slots,
             "weapons": [{"node": w.node, "slot": w.slot} for w in scn.weapons],
             "shots": shots, "pulses": pulses}
    (out_dir / "truth.json").write_text(json.dumps(truth) + "\n")
    return truth
