"""IT04 aim trace and after-action record (SPEC F07, F11, F12), through aim_host.

One weapon with a shaky hold (sway at 1.3 and 3.7 Hz) and a flinch before every
break: the muzzle dips 25 mm in the last 80 ms and recovers. 12 shots.
  F07  an AIM frame for (nearly) every aim pulse, at the weapon's slot rate,
       on the true aim point
  F12  one record per shot: aim trace from 1.0 s before the break to 0.5 s
       after, the hit, and S1's SHOT_DETAIL merged by shot number; the flinch
       is visible in the trace
  F11  TRK_STATUS at 1 Hz
"""
from __future__ import annotations

import math

import numpy as np

import lane as LN
import scenario as SC

T0 = 1000.0
BREAKS = [T0 + 1.2 + 0.7 * i for i in range(12)]
FLINCH_MM, FLINCH_S, RECOVER_S = 25.0, 0.08, 0.15


def hold(t: float) -> tuple[float, float]:
    """The shaky hold alone, without the flinch."""
    x = -200.0 + 8.0 * math.sin(2 * math.pi * 1.3 * (t - T0)) + 4.0 * math.sin(2 * math.pi * 3.7 * (t - T0) + 1.0)
    y = 150.0 + 6.0 * math.cos(2 * math.pi * 1.1 * (t - T0)) + 3.0 * math.sin(2 * math.pi * 3.1 * (t - T0))
    return x, y


def aim(t: float) -> tuple[float, float]:
    x, y = hold(t)
    for b in BREAKS:
        if b - FLINCH_S <= t < b:
            y -= FLINCH_MM * (t - (b - FLINCH_S)) / FLINCH_S
        elif b <= t < b + RECOVER_S:
            y -= FLINCH_MM * (1 - (t - b) / RECOVER_S)
    return x, y


def test_aim_trace_and_record(lib, cam, calibration, tmp_path):
    w = SC.Weapon(node=1, slot=0, aim=aim, breaks_s=BREAKS)
    scn = SC.Scenario(weapons=[w], t0_s=T0, duration_s=10.0, seed=41)
    truth, out = LN.run_scenario(scn, tmp_path / "it04", calibration["path"], cam)

    # F07: AIM against the true aim pulses.
    pulses = {p["k"] & 0xFFFF: p for p in truth["pulses"] if p["kind"] == "aim" and p["rendered"]}
    aims = {a["frame_seq"]: a for a in out["aims"]}
    found = [k for k in pulses if k in aims]
    err = np.array([math.hypot(aims[k]["x_mm"] - pulses[k]["x_mm"], aims[k]["y_mm"] - pulses[k]["y_mm"])
                    for k in found])
    extra = [k for k in aims if k not in pulses]

    # F12: records.
    shots = {s["shot_no"]: s for s in truth["shots"]}
    recs = {r["shot_no"]: r for r in out["aar"]}
    trace_pts, flinch, detail_ok, trace_err = [], [], 0, []
    aim_truth = sorted((p["t_mid_us"], p["x_mm"], p["y_mm"]) for p in truth["pulses"] if p["kind"] == "aim")
    for n, s in shots.items():
        r = recs[n]
        assert r["hit"]["result"] == "OK"
        detail_ok += all(abs(r["detail"][k] - v) < 1e-6 for k, v in s["detail"].items())
        tr = np.array(r["trace"])  # [ms from break, x, y]
        assert tr[:, 0].min() >= -1000.0 and tr[:, 0].max() <= 500.0
        trace_pts.append(len(tr))
        # The dip below the hold in the last 60 ms before the break (true dip: 9-22 mm, mean 16).
        last = tr[(tr[:, 0] >= -60) & (tr[:, 0] < 0)]
        dips = [hold((s["t_break_us"] + t_ms * 1000.0) * 1e-6)[1] - y for t_ms, _, y in last]
        flinch.append(float(np.mean(dips)) if dips else float("nan"))
        for t_ms, x, y in tr:
            t_us = s["t_break_us"] + t_ms * 1000.0
            j = min(range(len(aim_truth)), key=lambda i: abs(aim_truth[i][0] - t_us))
            trace_err.append(math.hypot(x - aim_truth[j][1], y - aim_truth[j][2]))
    st = out["status"]
    res = {
        "aim_pulses": len(pulses), "aim_frames": len(aims), "aim_found": len(found), "aim_extra": len(extra),
        "aim_error_mm": {"rms": float(np.sqrt(np.mean(err**2))), "max": float(err.max())},
        "aim_rate_hz": len(aims) / 10.0,
        "records": len(recs), "details_merged": int(detail_ok),
        "trace_points_per_record": {"min": min(trace_pts), "max": max(trace_pts)},
        "trace_error_mm_rms": float(np.sqrt(np.mean(np.square(trace_err)))),
        "flinch_seen_mm": {"min": float(np.nanmin(flinch)), "mean": float(np.nanmean(flinch))},
        "status_frames": len(st), "status_fps": [s["fps_x10"] for s in st],
        "status_worst_ms_max": max(s["worst_ms"] for s in st), "status_faults": sorted({s["faults"] for s in st}),
        "example_record": recs[6],
    }
    LN.write_result("it04_aim_trace.json", res)
    assert len(found) >= 0.99 * len(pulses)
    assert not extra, "no AIM without an aim pulse"
    assert res["aim_error_mm"]["rms"] <= 2.0
    assert len(recs) == len(shots) and detail_ok == len(shots)
    # Two slots: 50 Hz aim, so about 75 points in the 1.5 s window (fewer where shots replace aim pulses).
    assert min(trace_pts) >= 65
    assert res["trace_error_mm_rms"] <= 2.0
    assert res["flinch_seen_mm"]["min"] >= 10.0, "the flinch shows in every trace"
    assert 8 <= len(st) <= 11 and all(s["faults"] == 0 for s in st)
