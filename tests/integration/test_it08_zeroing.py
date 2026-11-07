"""IT08 zeroing (SPEC F13): the emitter is misaligned from the bore by a known
angle; five zeroing shots at the screen centre measure the offset, and the
corrected hits then land on the bore's aim point (P01) over the whole screen.

The shooter's aim scatter during zeroing (2 mm, a rested rifle at 5 m) is part
of the measured offset, as on a real lane.
"""
from __future__ import annotations

import ctypes as C

import numpy as np

import lane as LN

MIS_MRAD = (2.0, -1.0)  # emitter turned 2 mrad right and 1 mrad down from the bore
AIM_SCATTER_MM = 2.0
ZERO_SHOTS = 5
P01_RMS_MM = 2.0


def test_zeroing_removes_misalignment(lib, cam, calibration):
    lane = cam.lane
    rng = np.random.default_rng(8)
    pipe = LN.Pipeline(lib, cam, calibration["geom"], rng)
    zero = LN.AT.Zero()
    lib.at_zero_reset(C.byref(zero))

    # Zeroing: five shots at the target in the screen centre.
    target = np.array([0.0, 0.0])
    lib.at_zero_begin(C.byref(zero))
    for _ in range(ZERO_SHOTS):
        bore = target + rng.normal(0, AIM_SCATTER_MM, 2)
        hit, _ = pipe.shot(LN.camsim.emitter_point(bore, MIS_MRAD))
        assert hit is not None
        lib.at_zero_add(C.byref(zero), hit["x_mm"], hit["y_mm"], target[0], target[1])
    assert lib.at_zero_finish(C.byref(zero), ZERO_SHOTS, 20.0) == 0
    true_offset = LN.camsim.emitter_point(target, MIS_MRAD) - target

    # Shots over the screen: hit vs the bore's aim point, before and after the correction.
    margin = 40.0
    raw, corrected = [], []
    for _ in range(100):
        bore = rng.uniform([-lane.screen_w / 2 + margin, -lane.screen_h / 2 + margin],
                           [lane.screen_w / 2 - margin, lane.screen_h / 2 - margin])
        hit, _ = pipe.shot(LN.camsim.emitter_point(bore, MIS_MRAD))
        assert hit is not None
        x, y = C.c_double(hit["x_mm"]), C.c_double(hit["y_mm"])
        raw.append((x.value - bore[0], y.value - bore[1]))
        lib.at_zero_apply(C.byref(zero), C.byref(x), C.byref(y))
        corrected.append((x.value - bore[0], y.value - bore[1]))
    raw, corrected = np.array(raw), np.array(corrected)
    rms = lambda e: float(np.sqrt(np.mean(np.sum(e**2, axis=1))))  # noqa: E731
    res = {
        "misalignment_mrad": MIS_MRAD, "true_offset_at_centre_mm": true_offset.tolist(),
        "measured_offset_mm": [zero.off_x, zero.off_y], "zeroing_group_spread_mm": zero.spread_mm,
        "rms_before_mm": rms(raw), "rms_after_mm": rms(corrected),
        "mean_after_mm": corrected.mean(axis=0).tolist(), "max_after_mm": float(np.hypot(*corrected.T).max()),
    }
    LN.write_result("it08_zeroing.json", res)
    assert res["rms_before_mm"] > 5 * P01_RMS_MM, "the misalignment is visible before zeroing"
    # The offset is measured to within the mean aim error of five shots (3 sigma) plus 0.5 mm.
    assert np.hypot(*(np.array(res["measured_offset_mm"]) - true_offset)) < 3 * AIM_SCATTER_MM / np.sqrt(5) + 0.5
    assert res["rms_after_mm"] <= P01_RMS_MM
