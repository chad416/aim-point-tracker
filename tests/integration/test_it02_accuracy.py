"""IT02 single-weapon accuracy (SPEC F02-F04, P01): 500 shots spread over the
screen, steady aim, nominal emitter signal, after calibration. Hits are
measured against the simulator's ground truth.

P01: <= 2 mm RMS and <= 5 mm at the 99th percentile over the whole screen.
(HIT frames on CAN, F10, come with match in week 3.)
"""
from __future__ import annotations

import numpy as np

import lane as LN

N_SHOTS = 500
P01_RMS_MM = 2.0
P01_P99_MM = 5.0


def test_accuracy_over_screen(lib, cam, calibration):
    lane = cam.lane
    rng = np.random.default_rng(42)
    pipe = LN.Pipeline(lib, cam, calibration["geom"], rng)
    margin = 20.0  # mm: the whole spot (about 10 mm across) stays on the screen
    targets = rng.uniform([-lane.screen_w / 2 + margin, -lane.screen_h / 2 + margin],
                          [lane.screen_w / 2 - margin, lane.screen_h / 2 - margin], (N_SHOTS, 2))
    rows, missed = [], 0
    for t in targets:
        hit, truth = pipe.shot(t)
        if hit is None:
            missed += 1
            continue
        assert hit["status"] == "on_screen"
        rows.append((truth["x_mm"], truth["y_mm"], hit["x_mm"] - truth["x_mm"], hit["y_mm"] - truth["y_mm"]))
    a = np.array(rows)
    r = np.hypot(a[:, 2], a[:, 3])
    res = {
        "shots": N_SHOTS, "hits": len(rows), "missed": missed,
        "rms_mm": float(np.sqrt(np.mean(r**2))), "p99_mm": float(np.percentile(r, 99)), "max_mm": float(r.max()),
        "bias_mm": [float(a[:, 2].mean()), float(a[:, 3].mean())],
        "limits": {"rms_mm": P01_RMS_MM, "p99_mm": P01_P99_MM},
    }
    LN.write_result("it02_accuracy.json", res)
    np.savetxt(LN.RESULTS / "it02_hits.csv", a, delimiter=",", fmt="%.4f",
               header="true_x_mm,true_y_mm,err_x_mm,err_y_mm", comments="")
    assert missed == 0
    assert res["rms_mm"] <= P01_RMS_MM
    assert res["p99_mm"] <= P01_P99_MM
