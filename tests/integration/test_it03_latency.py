"""IT03 latency (SPEC F08, P02, P07), through aim_host.

One weapon, 100 shots at random break times over 10 s, slow aim drift.
  F08  every HIT comes from the right exposure (the first whose pulse starts
       >= 2 ms after the break, on the weapon's lane time) and is OK
  P02  HIT on CAN <= 25 ms after the break, in true lane time
  P07  every frame's exposure start, as aim_host computes it from SYNC and the
       camera timestamp, within +-100 us of the true one
"""
from __future__ import annotations

import math

import numpy as np

import lane as LN
import scenario as SC

N_SHOTS = 100
P02_MS = 25.0
P07_US = 100.0


def break_times(rng, t0: float, t1: float, n: int, min_gap: float) -> list[float]:
    """n breaks in [t0, t1], at least min_gap apart: min_gap plus exponential gaps, so the
    break falls anywhere within the 10 ms frame (the latency depends on that phase)."""
    spare = (t1 - t0) - n * min_gap
    gaps = min_gap + rng.exponential(spare / n, n)
    gaps *= min(1.0, (t1 - t0) / gaps.sum())
    return [float(v) for v in t0 + np.cumsum(gaps) - gaps[0]]


def test_latency_and_frame_times(lib, cam, calibration, tmp_path):
    rng = np.random.default_rng(31)
    t0 = 1000.0
    w = SC.Weapon(node=1, slot=0, aim=lambda t: (300 * math.sin(0.3 * (t - t0)), 200 * math.cos(0.2 * (t - t0))),
                  breaks_s=break_times(rng, t0 + 0.3, t0 + 9.7, N_SHOTS, 0.07))
    scn = SC.Scenario(weapons=[w], t0_s=t0, duration_s=10.0, seed=32)
    truth, out = LN.run_scenario(scn, tmp_path / "it03", calibration["path"], cam)
    clk = scn.clock

    hits = {(h["node"], h["shot_no"]): h for h in out["hits"]}
    aar = {(a["node"], a["shot_no"]): a for a in out["aar"]}
    lat, lat_field, pos_err, wrong_k = [], [], [], 0
    for s in truth["shots"]:
        h = hits[(s["node"], s["shot_no"])]
        assert h["result"] == 0, f"shot {s['shot_no']}: result {h['result']}"
        lane_hit = clk.lane0_us + (h["t_local_us"] - clk.local0_us) / (1 + clk.drift_ppm * 1e-6)
        lat.append((lane_hit - s["t_break_us"]) / 1000.0)
        lat_field.append(h["latency"])
        pos_err.append(math.hypot(h["x_mm"] - s["x_mm"], h["y_mm"] - s["y_mm"]))
        wrong_k += aar[(s["node"], s["shot_no"])]["k"] != s["k"]
    ks = {f["k"]: f["lane_t0_us"] for f in out["frames"]}
    p07 = [abs(t - (k * SC.FRAME_US + SC.EXPOSURE_START_US)) for k, t in ks.items()]
    lat = np.array(lat)
    res = {
        "shots": N_SHOTS, "hits_ok": int(sum(1 for h in out["hits"] if h["result"] == 0)),
        "wrong_frame": int(wrong_k),
        "latency_ms": {"mean": float(lat.mean()), "max": float(lat.max()), "min": float(lat.min())},
        "latency_field_vs_true_ms_max": float(np.max(np.abs(np.array(lat_field) - lat))),
        "hit_position_error_mm": {"rms": float(np.sqrt(np.mean(np.square(pos_err)))), "max": float(max(pos_err))},
        "frame_time_error_us": {"max": float(max(p07)), "rms": float(np.sqrt(np.mean(np.square(p07))))},
        "frames": len(ks), "proc_ms": out["summary"]["proc_ms"], "drift_ppm_estimated": out["summary"]["drift_ppm"],
        "drift_ppm_true": -clk.drift_ppm / (1 + clk.drift_ppm * 1e-6),
        "latencies_ms": [round(float(v), 3) for v in lat],
    }
    LN.write_result("it03_latency.json", res)
    assert len(out["hits"]) == N_SHOTS
    assert wrong_k == 0
    assert res["latency_ms"]["max"] <= P02_MS
    assert res["latency_field_vs_true_ms_max"] <= 0.2, "the HIT's own latency field matches true lane time"
    assert res["hit_position_error_mm"]["max"] <= 1.0
    assert res["frame_time_error_us"]["max"] <= P07_US
