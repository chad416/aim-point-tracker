"""Shared set-up for the integration tests (SPEC §9.3): the simulated lane, the
C library, and a shot pipeline (render -> detect -> map to the screen).

Until aim_host exists (week 3), the tests drive the C library through ctypes
on frames from the camera simulator; the pipeline is the same.
"""
from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results"
for p in ("sim", "tools/reference", "tools/calib"):
    sys.path.insert(0, str(ROOT / p))

import aimtrk_c as AT  # noqa: E402
import calibrate as CB  # noqa: E402, F401  (used by the tests as lane.CB)
import camsim  # noqa: E402

NOMINAL_PEAK_DN = 150.0   # the emitter design point (SPEC §6.2)
SPOT_SIGMA_PX = 1.2


def nominal_total_dn() -> float:
    return NOMINAL_PEAK_DN * 2 * math.pi * SPOT_SIGMA_PX**2


class Pipeline:
    """Frames from the camera simulator through the C detect and geometry."""

    def __init__(self, lib, cam: camsim.Camera, geom, rng: np.random.Generator):
        self.lib, self.cam, self.geom, self.rng = lib, cam, geom, rng
        self.det = AT.Detector(lib, AT.default_cfg(lib, cam.lane.width, cam.lane.height))
        self.k = 0
        bg, _ = cam.render([], self.k, rng)
        self.det.learn(bg)

    def shot(self, xy_mm, total_dn: float | None = None) -> tuple[dict | None, dict]:
        """One frame with one emitter pulse at xy_mm. Returns (hit or None, ground truth)."""
        self.k += 1
        spot = camsim.Spot(xy_mm=(float(xy_mm[0]), float(xy_mm[1])),
                           total_dn=total_dn if total_dn is not None else nominal_total_dn(), kind="shot")
        img, truth = self.cam.render([spot], self.k, self.rng)
        spots = self.det.frame(img)
        tr = truth[0]
        near = [s for s in spots if math.hypot(s["x"] - tr["u"], s["y"] - tr["v"]) < 3.0]
        if len(spots) != 1 or not near:
            return None, tr
        s = near[0]
        x, y, status = AT.to_screen(self.lib, self.geom, s["x"], s["y"])
        return {"x_mm": x, "y_mm": y, "status": status, "u": s["x"], "v": s["y"], "snr": s["snr"]}, tr


def write_result(name: str, data: dict) -> pathlib.Path:
    dst = RESULTS / name
    dst.write_text(json.dumps(data, indent=1) + "\n")
    return dst
