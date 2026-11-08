"""IT09 long run (SPEC P03, P05), through aim_host: 100 000 frames (1000 s of
lane time), two weapons sweeping the screen and firing about once a second
each, with the interference set at the typical projector leak (12 DN), a
sunlit window, glints, extra hot pixels and a person crossing the view.

  P03  >= 99.9 % of the visible shots give a HIT with a position on the true
       spot: within HIT_TOL_MM = 5 mm, P01's limit at the 99th percentile
       (the accuracy itself is P01's, reported here as the RMS of these
       hits); a false HIT is a position further than that from the true
       spot, or a position for a shot nobody could see: <= 1 in the
       100 000 frames. AMBIGUOUS HITs are counted apart (P04).
  P05  processing <= 3 ms per frame, worst case; no heap growth after
       start-up. Not shown here: on a virtual machine (WSL2, CI runners)
       time the host takes the CPU away still counts as the thread's CPU
       time, and the slowest frames by CPU time and by wall time are
       different frames. So the 99.9th percentile of the CPU time is
       asserted, and both clocks are reported with their maxima and the
       slowest frames (what was in them). A worst case needs a target board.

Slow (several minutes): excluded by default, run with  pytest -m long.
"""
from __future__ import annotations

import math
import re

import numpy as np
import pytest

import lane as LN
import scenario as SC

T0 = 1000.0
DURATION_S = 1000.0  # 100 000 frames
P05_MS = 3.0
HIT_TOL_MM = 5.0  # P01: 5 mm at the 99th percentile


def sweep(ax: float, ay: float, fx: float, fy: float, phase: float):
    def aim(t: float) -> tuple[float, float]:
        u = t - T0
        return (ax * math.sin(2 * math.pi * fx * u + phase) + 3 * math.sin(9 * t + phase),
                ay * math.sin(2 * math.pi * fy * u) + 3 * math.cos(7 * t + phase))
    return aim


@pytest.mark.long
def test_long_run(lib, calibration, tmp_path):
    cam = LN.camsim.Camera()
    rng = np.random.default_rng(90)
    breaks = []
    for _ in range(2):
        t, b = T0 + 1.0, []
        while t < T0 + DURATION_S - 1.0:
            b.append(t)
            t += float(rng.uniform(0.7, 1.3))
        breaks.append(b)
    w1 = SC.Weapon(node=1, slot=0, aim=sweep(1300, 700, 0.013, 0.021, 0.0), breaks_s=breaks[0])
    w2 = SC.Weapon(node=2, slot=1, aim=sweep(1250, 650, 0.017, 0.011, 2.0), breaks_s=breaks[1])
    interf = SC.Interference(leak_dn=12.0, window=True, glints=True, extra_hot=1e-4,
                             passer=(int(T0 * 100) + 50_000, int(T0 * 100) + 50_200))
    scn = SC.Scenario(weapons=[w1, w2], t0_s=T0, duration_s=DURATION_S, interference=interf, seed=91)
    truth, out = LN.run_scenario(scn, tmp_path / "it09", calibration["path"], cam)

    hits = {(h["node"], h["shot_no"]): h for h in out["hits"]}
    visible = ok = hidden = hidden_ok = ambiguous = false_hits = 0
    misses, errors = [], []
    for s in truth["shots"]:
        h = hits.get((s["node"], s["shot_no"]))
        if h is None:
            misses.append({"node": s["node"], "shot_no": s["shot_no"], "result": "no HIT"})
            continue
        if s.get("hidden") or not s.get("rendered", False):  # behind the person, on the window, out of view
            hidden += 1
            hidden_ok += h["result"] == 1
            false_hits += h["result"] in (0, 2, 4)
            continue
        if h["result"] == 3:
            ambiguous += 1
            continue
        visible += 1
        d = math.hypot(h["x_mm"] - s["x_mm"], h["y_mm"] - s["y_mm"])
        if h["result"] in (0, 2, 4) and d <= HIT_TOL_MM:
            ok += 1
            errors.append(d)
        else:
            if h["result"] in (0, 2, 4):
                false_hits += 1
            misses.append({"node": s["node"], "shot_no": s["shot_no"], "k": s["k"], "result": h["result"],
                           "d_mm": round(d, 2) if h["result"] in (0, 2, 4) else None})
    summ = out["summary"]
    res = {
        "frames": summ["frames"], "shots": len(truth["shots"]), "hits": len(out["hits"]),
        "visible": visible, "ok": ok, "detection_pct": 100.0 * ok / max(visible, 1),
        "hit_error_mm": {"rms": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                         "max": float(max(errors)) if errors else None},
        "hidden": hidden, "hidden_no_spot": hidden_ok, "ambiguous": ambiguous, "false_hits": false_hits,
        "aim_frames": len(out["aims"]), "stray_spots_per_1000_frames": 1000.0 * summ["match"]["stray_spots"] / summ["frames"],
        "proc_ms": summ["proc_ms"], "heap_bytes": summ["heap_bytes"],
        "frame_time_residual_max_us": summ["frame_time_residual_max_us"],
        "status_frames": len(out["status"]), "status_worst_ms_max": max(s["worst_ms"] for s in out["status"]),
        "misses": misses[:20],
    }
    # Processing time (thread CPU time): percentiles, and the slowest frames with what was in them, so a
    # spike can be told apart from a costly frame (WSL2 and CI runners are virtual machines: time the
    # host takes the CPU away still counts as this thread's CPU time).
    proc = np.array([f["proc_ms"] for f in out["frames"]])
    res["proc_ms_percentiles"] = {p: round(float(np.percentile(proc, p)), 3) for p in (50, 99, 99.9, 99.99)}
    wall = np.array([f["wall_ms"] for f in out["frames"]])
    res["wall_ms_percentiles"] = {p: round(float(np.percentile(wall, p)), 3) for p in (50, 99, 99.9, 99.99)}
    res["hit_tolerance_mm"] = HIT_TOL_MM
    slow = sorted(out["frames"], key=lambda f: -f["proc_ms"])[:8]
    res["slowest_frames"] = [{"k": f["k"], "proc_ms": f["proc_ms"], "wall_ms": f["wall_ms"], "spots": f["spots"],
                              "rejected": f["rejected"]} for f in slow]
    res["slowest_frames_by_wall"] = [{"k": f["k"], "proc_ms": f["proc_ms"], "wall_ms": f["wall_ms"],
                                      "spots": f["spots"]} for f in sorted(out["frames"], key=lambda f: -f["wall_ms"])[:8]]
    typical = [f for f in out["frames"] if f["proc_ms"] <= np.percentile(proc, 50)]
    res["typical_frame_rejected_mean"] = [round(float(np.mean([f["rejected"][i] for f in typical])), 1)
                                          for i in range(4)]
    res["reacquired"] = summ["match"]["reacquired"]
    res["untracked_shots"] = summ["match"]["untracked_shots"]
    res["sync"] = summ["sync"]
    LN.write_result("it09_long_run.json", res)

    assert summ["frames"] >= 99_900
    assert ok >= 0.999 * visible
    assert false_hits <= 1
    assert hidden_ok == hidden
    # P05 on the 99.9th percentile; the maximum is reported (see slowest_frames and SPEC 9.4).
    assert res["proc_ms_percentiles"][99.9] <= P05_MS
    assert summ["heap_bytes"]["end"] == summ["heap_bytes"]["after_first_frame"], "no heap growth"


def test_library_has_no_heap():
    """P05: the C library itself never allocates (aim_host allocates its buffers once at start-up)."""
    calls = []
    for p in sorted((LN.ROOT / "aimtrk" / "src").glob("*.c")):
        for i, line in enumerate(p.read_text().splitlines(), 1):
            if re.search(r"\b(malloc|calloc|realloc|free)\s*\(", line):
                calls.append(f"{p.name}:{i}")
    assert not calls, calls
