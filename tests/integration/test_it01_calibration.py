"""IT01 calibration (SPEC F05, P06): grid capture, fit, residual and hold-out
check, calib.json with version and CRC; a bad grid is refused."""
from __future__ import annotations

import json

import numpy as np
import pytest

import lane as LN

P06_FIT_PX = 0.3
P06_HOLDOUT_PX = 0.5


def test_grid_fit_meets_p06(calibration, cam, lib):
    rep = calibration["report"]
    assert rep["status"] == "ok"
    assert rep["n_found"] == 54 and rep["n_holdout"] == 10 and rep["n_fit"] == 44
    assert rep["fit_rms_px"] <= P06_FIT_PX
    assert rep["holdout_rms_px"] <= P06_HOLDOUT_PX
    # The geometry against the simulator's true camera, over the whole screen.
    lane = cam.lane
    rng = np.random.default_rng(1)
    pts = rng.uniform([-lane.screen_w / 2, -lane.screen_h / 2], [lane.screen_w / 2, lane.screen_h / 2], (2000, 2))
    err = []
    for p in pts:
        u, v = lane.screen_to_image(p)[0]
        x, y, _ = LN.AT.to_screen(lib, calibration["geom"], u, v)
        err.append(np.hypot(x - p[0], y - p[1]))
    err = np.array(err)
    # Reprojection residual of every grid dot (for the plot), held-out dots marked.
    dots = calibration["dots"]
    residuals = []
    for i, (gx, gy) in enumerate(calibration["grid"]):
        u, v = LN.AT.to_image(lib, calibration["geom"], gx, gy)
        j = int(np.argmin(np.hypot(dots[:, 0] - u, dots[:, 1] - v)))
        residuals.append([float(gx), float(gy), float(dots[j, 0] - u), float(dots[j, 1] - v), int(i % 5 == 4)])
    LN.write_result("it01_calibration.json", {
        "residuals_px": residuals,
        "report": rep, "nominal_lens": LN.camsim.NOMINAL_LENS,
        "true_lens": {k: getattr(lane, k) for k in ("f_px", "k1", "k2", "p1", "p2")},
        "true_principal_point": [lane.cx, lane.cy],
        "fitted_lens": calibration["doc"]["geometry"]["lens"],
        "geometry_error_mm": {"rms": float(np.sqrt(np.mean(err**2))), "p99": float(np.percentile(err, 99)),
                              "max": float(err.max())},
    })
    assert err.max() < 0.25, "geometry alone must be far inside P01"


def test_calib_json_crc(calibration, lib, tmp_path):
    doc = json.loads(calibration["path"].read_text())
    assert doc["version"] == LN.CB.VERSION
    bad = dict(doc)
    bad["geometry"] = dict(doc["geometry"], screen_w=doc["geometry"]["screen_w"] + 1.0)
    p = tmp_path / "calib.json"
    p.write_text(json.dumps(bad))
    with pytest.raises(ValueError, match="CRC"):
        LN.CB.load(lib, p)
    p.write_text(json.dumps(dict(doc, version=99)))
    with pytest.raises(ValueError, match="version"):
        LN.CB.load(lib, p)


def test_displaced_grid_refused(lib, cam):
    """The projector puts the dots 8 mm (about 3 px) away from where the grid says: residual over P06."""
    rng = np.random.default_rng(21)
    grid = LN.camsim.calibration_grid(cam.lane)
    shown = grid + rng.normal(0, 8.0, grid.shape)
    bg, img = LN.CB.capture_grid(cam, rng, grid, spots=LN.camsim.grid_spots(shown))
    dots = LN.CB.detect_dots(lib, cam, bg, img)
    geom, rep = LN.CB.calibrate_from_dots(lib, dots, grid, cam.lane)
    assert geom is None and rep["status"] == "residual", rep
    assert rep["fit_rms_px"] > P06_FIT_PX


def test_blocked_grid_refused(lib, cam):
    """A person in front of the screen hides a block of dots: too few found, refused."""
    rng = np.random.default_rng(22)
    grid = LN.camsim.calibration_grid(cam.lane)
    visible = grid[~((grid[:, 0] > -200) & (grid[:, 0] < 400) & (grid[:, 1] < 200))]
    bg, img = LN.CB.capture_grid(cam, rng, grid, spots=LN.camsim.grid_spots(visible))
    dots = LN.CB.detect_dots(lib, cam, bg, img)
    assert len(dots) < len(grid)
    geom, rep = LN.CB.calibrate_from_dots(lib, dots, grid, cam.lane)
    assert geom is None and rep["status"] == "too_few", rep
