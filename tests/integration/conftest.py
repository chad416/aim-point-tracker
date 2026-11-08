"""Session fixtures: the C library, the simulated camera, and one calibration
of the lane (IT01 checks it; the other tests use it, as a lane would).

The calibration is written to a temporary directory, so a test run does not
rewrite results/calib.json (made by tools/calib/calibrate.py; it holds the
time it was made). Import paths (sim/, tools/...) are set in pytest.ini."""
from __future__ import annotations

import numpy as np
import pytest

import lane as LN


@pytest.fixture(scope="session")
def lib():
    return LN.AT.load()


@pytest.fixture(scope="session")
def cam():
    return LN.camsim.Camera()


@pytest.fixture(scope="session")
def calibration(lib, cam, tmp_path_factory):
    """Capture the grid, detect, fit; store calib.json and load it back (CRC checked)."""
    rng = np.random.default_rng(5)
    grid = LN.camsim.calibration_grid(cam.lane)
    bg, img = LN.CB.capture_grid(cam, rng, grid)
    dots = LN.CB.detect_dots(lib, cam, bg, img)
    geom, rep = LN.CB.calibrate_from_dots(lib, dots, grid, cam.lane)
    assert geom is not None, f"calibration refused: {rep}"
    path = tmp_path_factory.mktemp("calib") / "calib.json"
    LN.CB.save(path, geom, rep, grid)
    loaded, doc = LN.CB.load(lib, path)
    return {"geom": loaded, "report": rep, "dots": dots, "grid": grid, "path": path, "doc": doc}
