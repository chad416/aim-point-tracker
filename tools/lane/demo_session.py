#!/usr/bin/env python3
"""A scripted two-weapon training session through aim_host (SPEC §10).

Two shooters on one lane, each engaging two targets on the screen: the aim
swings onto the target, settles with a little sway, and three shots follow,
each with a small flinch before the break. Weapon 2 is mounted with its
emitter 2 mrad off the bore and is zeroed first (aim_host --zero), so its
hits are corrected. The camera simulator renders the session and streams it
into aim_host; the session's outputs (CAN log, after-action records, summary)
go to docs/session/ for the lane view.

Usage: demo_session.py [--out docs/session]
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
for p in ("sim", "tools/lane", "tools/reference", "tools/calib"):
    sys.path.insert(0, str(ROOT / p))

import calibrate as CB  # noqa: E402
import camsim  # noqa: E402
import numpy as np  # noqa: E402
import runner  # noqa: E402
import scenario as SC  # noqa: E402

T0 = 1000.0
TARGETS = {"A": (-900.0, 300.0), "B": (-350.0, -280.0), "C": (450.0, 330.0), "D": (1000.0, -220.0)}
TARGET_RADIUS_MM = 75.0


def engagement(plan, start_xy, mis_flinch=22.0):
    """Aim path: [(t_arrive, t_leave, target_xy, break_times)]: swing on, hold with sway, flinch before breaks."""
    def aim(t: float) -> tuple[float, float]:
        x, y = start_xy
        for i, (t_on, t_off, (tx, ty), breaks) in enumerate(plan):
            prev = plan[i - 1][2] if i > 0 else start_xy
            if t < t_on - 0.6:  # between targets: still on the previous one (or the start point)
                x, y = prev
                break
            if t < t_on:  # swing on over 0.6 s (smooth)
                a = 0.5 - 0.5 * math.cos(math.pi * (t - (t_on - 0.6)) / 0.6)
                x, y = prev[0] + a * (tx - prev[0]), prev[1] + a * (ty - prev[1])
                break
            if t <= t_off or i == len(plan) - 1:
                x = tx + 5 * math.sin(2 * math.pi * 1.1 * t + i) + 2 * math.sin(2 * math.pi * 3.3 * t)
                y = ty + 4 * math.cos(2 * math.pi * 0.9 * t + i) + 2 * math.cos(2 * math.pi * 2.9 * t)
                for b in breaks:
                    if b - 0.07 <= t < b:
                        y -= mis_flinch * (t - (b - 0.07)) / 0.07
                    elif b <= t < b + 0.15:
                        y -= mis_flinch * (1 - (t - b) / 0.15)
                break
        return x, y
    return aim


def build() -> tuple[SC.Scenario, dict]:
    p1 = [(T0 + 1.2, T0 + 4.2, TARGETS["A"], [T0 + 2.4, T0 + 3.0, T0 + 3.6]),
          (T0 + 5.6, T0 + 9.0, TARGETS["B"], [T0 + 6.8, T0 + 7.5, T0 + 8.2])]
    p2 = [(T0 + 1.8, T0 + 5.0, TARGETS["C"], [T0 + 3.0, T0 + 3.9, T0 + 4.5]),  # 3.0 s: same frame as weapon 1
          (T0 + 6.4, T0 + 10.0, TARGETS["D"], [T0 + 7.6, T0 + 8.4, T0 + 9.1])]
    w1 = SC.Weapon(node=1, slot=0, aim=engagement(p1, (-200.0, -600.0)),
                   breaks_s=[b for _, _, _, bs in p1 for b in bs])
    w2 = SC.Weapon(node=2, slot=1, aim=engagement(p2, (300.0, -650.0), mis_flinch=12.0),
                   breaks_s=[b for _, _, _, bs in p2 for b in bs], mis_mrad=(2.0, 1.0))
    scn = SC.Scenario(weapons=[w1, w2], t0_s=T0, duration_s=10.6, seed=7,
                      interference=SC.Interference(leak_dn=8.0, glints=True))
    meta = {"targets": {k: list(v) for k, v in TARGETS.items()}, "target_radius_mm": TARGET_RADIUS_MM,
            "weapons": [{"node": 1, "slot": 0, "targets": ["A", "B"]},
                        {"node": 2, "slot": 1, "targets": ["C", "D"], "emitter_mis_mrad": [2.0, 1.0]}],
            "screen_mm": [3000.0, 1690.0]}
    return scn, meta


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(ROOT / "docs" / "session"))
    args = ap.parse_args(argv)
    out = pathlib.Path(args.out)
    work = ROOT / "build" / "demo"
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)

    # Calibrate the lane (as a lane would at start-up).
    import aimtrk_c as AT
    lib = AT.load()
    cam = camsim.Camera()
    rng = np.random.default_rng(5)
    grid = camsim.calibration_grid(cam.lane)
    bg, img = CB.capture_grid(cam, rng, grid)
    geom, rep = CB.calibrate_from_dots(lib, CB.detect_dots(lib, cam, bg, img), grid, cam.lane)
    if geom is None:
        print("calibration refused", rep)
        return 1
    calib = work / "calib.json"
    CB.save(calib, geom, rep, grid)

    # Zero weapon 2 (emitter 2 mrad right, 1 mrad up of the bore): five shots at the screen centre.
    zero = SC.Weapon(node=2, slot=1, aim=lambda t: (0.0 + 1.5 * math.sin(7 * t), 0.0 + 1.5 * math.cos(5 * t)),
                     breaks_s=[T0 + 0.6 + 0.4 * i for i in range(5)], mis_mrad=(2.0, 1.0))
    zscn = SC.Scenario(weapons=[SC.Weapon(node=1, slot=0, aim=lambda t: (0.0, -700.0), raised=lambda t: False),
                                zero], t0_s=T0, duration_s=2.8, seed=8)
    runner.run_scenario(zscn, work / "zeroing", calib, cam, "--zero", "2:0:0:5", "--zero-out",
                        str(work / "zero.txt"))
    print("zeroing:", (work / "zero.txt").read_text().strip())

    # The session.
    scn, meta = build()
    truth, res = runner.run_scenario(scn, work / "session", calib, camsim.Camera(), "--zero-in", str(work / "zero.txt"))
    out.mkdir(parents=True, exist_ok=True)
    for name in ("can_out.log", "aar.jsonl", "summary.json", "can_in.log"):
        shutil.copy(work / "session" / name, out / name)
    meta["zero_txt"] = (work / "zero.txt").read_text().strip()
    meta["truth_shots"] = [{k: s.get(k) for k in ("node", "shot_no", "k", "t_break_us", "x_mm", "y_mm")}
                           for s in truth["shots"]]
    (out / "session.json").write_text(json.dumps(meta, indent=1) + "\n")
    print(json.dumps(res["summary"]))
    print("->", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
