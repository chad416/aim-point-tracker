#!/usr/bin/env python3
"""Calibration runner (SPEC F05, §6.4).

Captures the projected 9 x 6 dot grid with the camera simulator, detects the
dots with the C library, fits the geometry (at_calib_run) and stores it in
calib.json with a version and two CRC-32s:
  crc32           the whole body (canonical JSON), checked by load() here;
  geometry_crc32  the 18 numbers aim_host uses, as little-endian doubles, so
                  the C reader (host/calibfile.c) can check them too.
load() refuses a file whose CRCs or version do not match.

Usage: calibrate.py [--out results/calib.json] [--seed 5]
"""
from __future__ import annotations

import argparse
import json
import pathlib
import struct
import sys
import time
import zlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "tools" / "reference"))

import aimtrk_c as AT  # noqa: E402
import camsim  # noqa: E402

VERSION = 2
LENS_KEYS = ("f", "cx", "cy", "k1", "k2", "p1", "p2")


def nominal_lens() -> AT.Lens:
    n = camsim.NOMINAL_LENS
    return AT.lens_from(n["f"], n["cx"], n["cy"])


def capture_grid(cam: camsim.Camera, rng: np.random.Generator, grid_mm: np.ndarray,
                 spots: list | None = None) -> tuple[np.ndarray, np.ndarray]:
    """A background frame and a grid frame (the projector shows the grid in the second)."""
    bg, _ = cam.render([], 0, rng)
    img, _ = cam.render(spots if spots is not None else camsim.grid_spots(grid_mm), 1, rng)
    return bg, img


def detect_dots(lib, cam: camsim.Camera, bg: np.ndarray, img: np.ndarray) -> np.ndarray:
    det = AT.Detector(lib, AT.default_cfg(lib, cam.lane.width, cam.lane.height))
    det.learn(bg)
    return np.array([(s["x"], s["y"]) for s in det.frame(img)]).reshape(-1, 2)


def calibrate_from_dots(lib, dots: np.ndarray, grid_mm: np.ndarray, lane: camsim.Lane):
    return AT.calibrate(lib, dots, grid_mm, nominal_lens(), lane.screen_w, lane.screen_h)


def crc(body: dict) -> int:
    return zlib.crc32(json.dumps(body, sort_keys=True, separators=(",", ":")).encode())


def geometry_crc(g: dict) -> int:
    """CRC-32 of lens (7), to_cam (9), screen_w, screen_h as little-endian doubles (host/calibfile.c)."""
    vals = [g["lens"][k] for k in LENS_KEYS] + list(g["to_cam"]) + [g["screen_w"], g["screen_h"]]
    return zlib.crc32(struct.pack("<18d", *vals))


def save(path: pathlib.Path, geom, report: dict, grid_mm: np.ndarray) -> dict:
    g = AT.geom_dict(geom)
    body = {"version": VERSION, "created_unix": int(time.time()), "geometry": g,
            "geometry_crc32": geometry_crc(g), "report": report,
            "grid_mm": np.asarray(grid_mm).round(3).tolist(), "nominal_lens": camsim.NOMINAL_LENS}
    doc = {**body, "crc32": crc(body)}
    path.write_text(json.dumps(doc, indent=1) + "\n")
    return doc


def load(lib, path: pathlib.Path):
    """Geometry from calib.json; ValueError if the version or the CRC is wrong."""
    doc = json.loads(path.read_text())
    body = {k: v for k, v in doc.items() if k != "crc32"}
    if doc.get("version") != VERSION:
        raise ValueError(f"calib.json version {doc.get('version')}, expected {VERSION}")
    if crc(body) != doc.get("crc32") or geometry_crc(doc["geometry"]) != doc.get("geometry_crc32"):
        raise ValueError("calib.json CRC mismatch")
    return AT.geom_from_dict(lib, doc["geometry"]), doc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(ROOT / "results" / "calib.json"))
    ap.add_argument("--seed", type=int, default=5)
    args = ap.parse_args(argv)
    lib = AT.load()
    cam = camsim.Camera()
    rng = np.random.default_rng(args.seed)
    grid = camsim.calibration_grid(cam.lane)
    bg, img = capture_grid(cam, rng, grid)
    dots = detect_dots(lib, cam, bg, img)
    geom, rep = calibrate_from_dots(lib, dots, grid, cam.lane)
    print(f"{len(dots)} dots; calibration {rep['status']}: fit {rep['fit_rms_px']:.4f} px RMS, "
          f"held-out {rep['holdout_rms_px']:.4f} px RMS, worst {rep['max_px']:.4f} px, "
          f"{rep['iterations']} iterations")
    if geom is None:
        print("refused: previous calibration stays in use")
        return 1
    save(pathlib.Path(args.out), geom, rep, grid)
    print("->", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
