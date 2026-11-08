#!/usr/bin/env python3
"""Cross-check the C detect/centroid (host shared library) against the NumPy
reference on full 1280 x 800 frames from the camera simulator (SPEC §9.2).

The sequence mixes weak, nominal and saturated spots on the screen, moving
spots (smear), spots at the image border, a large bright patch (a reflection),
a bright line (an edge glint) and the sensor's hot pixels, and lets the
background run over the frames, so every rejection rule and the background
freeze are exercised. Both implementations must agree frame by frame:

  spots       same count and order; x, y within 2e-4 px (float32 vs double);
              area, saturated pixels and flags equal; sum, SNR, sizes to 1e-5
  stats       noise sigma, blob count, rejections and frozen cells equal
  background  equal after the last frame

It also reports the C spots against the simulator's ground truth and the C
processing time per frame (first look at P05), and runs geom_check.py: the C
lens model, homography and calibration against OpenCV.

Usage: crosscheck.py [--frames 40] [--seed 3]   -> results/crosscheck.json
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "sim"))

import aimtrk_c as AT  # noqa: E402
import camsim  # noqa: E402
import geom_check  # noqa: E402
from detect_ref import DetectRef  # noqa: E402

TOL_PX = 2e-4
TOL_REL = 1e-5


def scene(cam: camsim.Camera, rng: np.random.Generator, k: int) -> tuple[list, list]:
    """Screen spots and image-space extras (callables on the signal) for frame k."""
    lane = cam.lane
    spots = []
    for _ in range(6):
        total = float(np.exp(rng.uniform(np.log(150.0), np.log(30_000.0))))  # weak .. saturated
        s = camsim.random_spots(rng, lane, 1, total)[0]
        if rng.random() < 0.5:  # moving aim, up to 2 m/s on the screen (smear up to 1 mm in 0.5 ms)
            s.vel_mm_s = tuple(rng.uniform(-2000, 2000, 2))
            s.t_us = (camsim.exposure_window(k)[0] + 250, camsim.exposure_window(k)[0] + 750)
        spots.append(s)
    extras = []
    # Spots at the image border (the screen itself is 35 px inside the frame).
    for u, v in ((0.3, rng.uniform(50, 750)), (rng.uniform(50, 1230), 799.2)):
        extras.append(lambda sig, u=u, v=v: camsim.render_gaussian(sig, [(u, v)], 1.44 * np.eye(2), 2000.0))
    if k % 5 == 2:  # a reflection: large and bright, with a spot on top of it (detect step 6)
        x0, y0 = int(rng.integers(100, 1100)), int(rng.integers(100, 600))
        extras.append(lambda sig, x0=x0, y0=y0: sig.__setitem__((slice(y0, y0 + 30), slice(x0, x0 + 40)),
                                                                 sig[y0:y0 + 30, x0:x0 + 40] + 40.0))
        u, v = x0 + float(rng.uniform(12, 28)), y0 + float(rng.uniform(10, 20))
        extras.append(lambda sig, u=u, v=v: camsim.render_gaussian(sig, [(u, v)], 1.44 * np.eye(2), 1300.0))
    if k % 5 == 3:  # a glint: a thin bright line
        x0, y0 = int(rng.integers(100, 1100)), int(rng.integers(100, 600))
        extras.append(lambda sig, x0=x0, y0=y0: sig.__setitem__((slice(y0, y0 + 2), slice(x0, x0 + 18)),
                                                                 sig[y0:y0 + 2, x0:x0 + 18] + 60.0))
    return spots, extras


def render(cam: camsim.Camera, rng, k: int, with_spots: bool):
    exposure = camsim.exposure_window(k)
    sig = np.array(cam.ambient, dtype=np.float32)
    truth = []
    if with_spots:
        spots, extras = scene(cam, rng, k)
        truth = [cam.add_spot(sig, s, exposure) for s in spots]
        for e in extras:
            e(sig)
    return cam.expose(sig, rng), truth


def close(a: float, b: float, rel: float) -> bool:
    return abs(a - b) <= rel * max(abs(a), abs(b), 1.0)


def compare(fc: list[dict], fr: list[dict], k: int) -> list[str]:
    errs = []
    if len(fc) != len(fr):
        return [f"frame {k}: {len(fc)} C spots vs {len(fr)} reference"]
    for i, (c, r) in enumerate(zip(fc, fr)):
        for f in ("x", "y", "cx", "cy"):
            if abs(c[f] - r[f]) > TOL_PX:
                errs.append(f"frame {k} spot {i}: {f} C {c[f]:.6f} ref {r[f]:.6f}")
        for f in ("area", "n_sat", "flags"):
            if c[f] != r[f]:
                errs.append(f"frame {k} spot {i}: {f} C {c[f]} ref {r[f]}")
        for f in ("sum", "peak", "snr", "sig_major", "sig_minor"):
            if not close(c[f], r[f], TOL_REL):
                errs.append(f"frame {k} spot {i}: {f} C {c[f]:.6g} ref {r[f]:.6g}")
    return errs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--seed", type=int, default=3)
    args = ap.parse_args(argv)

    lib = AT.load()
    cam = camsim.Camera()
    w, h = cam.lane.width, cam.lane.height
    det = AT.Detector(lib, AT.default_cfg(lib, w, h))
    ref = DetectRef(w, h)
    rng = np.random.default_rng(args.seed)

    errors, times, matched, totals = [], [], [], {"spots": 0, "truth": 0}
    rej_total = dict.fromkeys(AT.REJECT_NAMES, 0)
    sat_spots = rim_spots = recovered = 0
    for k in range(args.frames):
        img, truth = render(cam, rng, k, with_spots=k > 0)
        t = time.perf_counter()
        sc = det.frame(img)
        times.append(time.perf_counter() - t)
        sr = ref.frame(img)
        st = det.stats()
        if k == 0:
            if not st["learning"] or sc:
                errors.append("frame 0 did not seed the background")
            continue
        errors += compare(sc, sr, k)
        rs = ref.stats
        if st["overflow"]:
            errors.append(f"frame {k}: table overflow {st['overflow']}")
        for f in ("sigma", "blobs", "frozen_cells", "snapped_cells", "recovered"):
            if st[f] != rs[f]:
                errors.append(f"frame {k}: stats {f} C {st[f]} ref {rs[f]}")
        if st["rejected"] != rs["rejected"]:
            errors.append(f"frame {k}: rejections C {st['rejected']} ref {rs['rejected']}")
        for n, v in st["rejected"].items():
            rej_total[n] += v
        totals["spots"] += len(sc)
        totals["truth"] += len(truth)
        sat_spots += sum(1 for s in sc if s["flags"] & AT.AT_SPOT_SATURATED)
        rim_spots += sum(1 for s in sc if s["flags"] & AT.AT_SPOT_RIM)
        recovered += st["recovered"]
        # Ground truth: nearest C spot within 2 px of each screen spot.
        for tr in truth:
            if not sc:
                continue
            d = [math.hypot(s["x"] - tr["u"], s["y"] - tr["v"]) for s in sc]
            i = int(np.argmin(d))
            if d[i] < 2.0:
                matched.append({"err_px": d[i], "peak_dn": tr["peak_dn"],
                                "sat": bool(sc[i]["flags"] & AT.AT_SPOT_SATURATED)})

    bg_c = det.bg()
    bg_r = (ref.bg256 / 256.0).astype(np.float32)
    if not np.array_equal(bg_c, bg_r):
        errors.append(f"background differs in {(bg_c != bg_r).sum()} cells")

    geo, geo_errs = geom_check.check(lib, cam.lane, np.random.default_rng(args.seed + 100))
    errors += geo_errs

    detectable = [m for m in matched if not m["sat"]]
    ms = np.array(times[1:]) * 1e3
    res = {
        "frames": args.frames, "seed": args.seed, "mismatches": len(errors),
        "spots_compared": totals["spots"], "screen_spots": totals["truth"],
        "screen_spots_found": len(matched), "saturated": sat_spots, "rim_fits": rim_spots,
        "recovered_in_large_blobs": recovered,
        "rejected": rej_total,
        "truth_rms_px_unsaturated": round(float(np.sqrt(np.mean([m["err_px"] ** 2 for m in detectable]))), 4)
        if detectable else None,
        "c_ms_per_frame": {"mean": round(float(ms.mean()), 3), "max": round(float(ms.max()), 3)},
        "geometry_vs_opencv": geo,
    }
    dst = HERE.parents[1] / "results" / "crosscheck.json"
    dst.write_text(json.dumps(res, indent=2) + "\n")
    for e in errors[:20]:
        print("MISMATCH", e)
    print(json.dumps(res, indent=2))
    print("crosscheck", "FAILED" if errors else "passed", f"-> {dst}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
