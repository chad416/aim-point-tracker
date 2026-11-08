"""ctypes bindings for the aimtrk C library, built as a host shared library.

Used by crosscheck.py (C vs the NumPy reference) and centroid_curve.py.
Structures mirror aimtrk/include/aimtrk/*.h; keep them in step. The at_detect
state itself is opaque here (a byte buffer); stats are read through
at_detect_get_stats().
"""
from __future__ import annotations

import ctypes as C
import pathlib
import subprocess

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
LIB = ROOT / "build" / "libaimtrk.so"

AT_SPOT_SATURATED = 0x01
AT_SPOT_RIM = 0x02
AT_SPOT_RIM_FAILED = 0x04
AT_SPOT_EDGE = 0x08
REJECT_NAMES = ("small", "large", "weak", "elongated")


class DetectCfg(C.Structure):
    _fields_ = [("width", C.c_uint16), ("height", C.c_uint16), ("bg_shift", C.c_uint8),
                ("k_sigma", C.c_float), ("t_min", C.c_float), ("min_area", C.c_uint16),
                ("max_area", C.c_uint16), ("max_extent", C.c_uint16), ("max_elong", C.c_float),
                ("min_snr", C.c_float), ("sat_level", C.c_uint8), ("max_runs", C.c_uint16),
                ("max_blobs", C.c_uint16)]


class Spot(C.Structure):
    _fields_ = [(n, C.c_float) for n in ("x", "y", "cx", "cy", "sum", "peak", "snr",
                                         "sig_major", "sig_minor")] + [
        ("area", C.c_uint16), ("n_sat", C.c_uint16), ("flags", C.c_uint8)]

    def as_dict(self) -> dict:
        return {n: getattr(self, n) for n, _ in self._fields_}


class Stats(C.Structure):
    _fields_ = [("sigma", C.c_float), ("bg_mean", C.c_float), ("runs", C.c_uint32),
                ("blobs", C.c_uint32), ("spots", C.c_uint32), ("rejected", C.c_uint32 * 4),
                ("frozen_cells", C.c_uint32), ("snapped_cells", C.c_uint32), ("recovered", C.c_uint32),
                ("overflow", C.c_uint8), ("learning", C.c_bool)]


class CWin(C.Structure):
    _fields_ = [("px", C.POINTER(C.c_uint8)), ("stride", C.c_size_t), ("x0", C.c_int),
                ("y0", C.c_int), ("w", C.c_int), ("h", C.c_int), ("mask", C.POINTER(C.c_uint8)),
                ("bg", C.POINTER(C.c_float)), ("bg_const", C.c_float)]


def build(lib: pathlib.Path = LIB) -> None:
    lib.parent.mkdir(parents=True, exist_ok=True)
    src = sorted(str(p) for p in (ROOT / "aimtrk" / "src").glob("*.c"))
    subprocess.run(["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Wconversion", "-Wdouble-promotion", "-Werror",
                    "-shared", "-fPIC", "-I", str(ROOT / "aimtrk" / "include"), *src, "-lm",
                    "-o", str(lib)], check=True)


def load(lib: pathlib.Path = LIB, rebuild: bool = True) -> C.CDLL:
    if rebuild or not lib.exists():
        build(lib)
    L = C.CDLL(str(lib))
    L.at_detect_cfg_default.argtypes = [C.POINTER(DetectCfg), C.c_uint16, C.c_uint16]
    L.at_detect_mem_size.argtypes = [C.POINTER(DetectCfg)]
    L.at_detect_mem_size.restype = C.c_size_t
    L.at_detect_init.argtypes = [C.c_void_p, C.POINTER(DetectCfg), C.c_void_p, C.c_size_t]
    L.at_detect_learn.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t]
    L.at_detect_frame.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t, C.POINTER(Spot), C.c_int]
    L.at_detect_get_stats.argtypes = [C.c_void_p]
    L.at_detect_get_stats.restype = C.POINTER(Stats)
    L.at_detect_bg.argtypes = [C.c_void_p, C.c_int, C.c_int]
    L.at_detect_bg.restype = C.c_float
    L.at_centroid.argtypes = [C.POINTER(CWin), C.c_float, C.c_uint8, C.POINTER(Spot)]
    setup_geom(L)
    return L


def default_cfg(lib: C.CDLL, width: int, height: int, **over) -> DetectCfg:
    cfg = DetectCfg()
    lib.at_detect_cfg_default(C.byref(cfg), width, height)
    for k, v in over.items():
        setattr(cfg, k, v)
    return cfg


class Detector:
    """One at_detect instance with its memory block."""

    MAX_SPOTS = 64

    def __init__(self, lib: C.CDLL, cfg: DetectCfg):
        self.lib, self.cfg = lib, cfg
        size = lib.at_detect_mem_size(C.byref(cfg))
        if size == 0:
            raise ValueError("invalid configuration")
        self.mem = C.create_string_buffer(size)
        self.state = C.create_string_buffer(4096)  # >= sizeof(at_detect)
        if lib.at_detect_init(self.state, C.byref(cfg), self.mem, size) != 0:
            raise ValueError("at_detect_init failed")
        self.spots = (Spot * self.MAX_SPOTS)()

    def learn(self, img: np.ndarray) -> None:
        img = np.ascontiguousarray(img, dtype=np.uint8)
        self.lib.at_detect_learn(self.state, img.ctypes.data, img.strides[0])

    def frame(self, img: np.ndarray) -> list[dict]:
        img = np.ascontiguousarray(img, dtype=np.uint8)
        n = self.lib.at_detect_frame(self.state, img.ctypes.data, img.strides[0], self.spots,
                                     self.MAX_SPOTS)
        return [self.spots[i].as_dict() for i in range(n)]

    def stats(self) -> dict:
        s = self.lib.at_detect_get_stats(self.state).contents
        d = {n: getattr(s, n) for n, _ in Stats._fields_ if n != "rejected"}
        d["rejected"] = dict(zip(REJECT_NAMES, list(s.rejected)))
        return d

    def bg(self) -> np.ndarray:
        """Background per cell, DN (for checks)."""
        cw, ch = self.cfg.width // 4, self.cfg.height // 4
        return np.array([[self.lib.at_detect_bg(self.state, 4 * c, 4 * r) for c in range(cw)]
                         for r in range(ch)], dtype=np.float32)


# ---------------------------------------------------------------- geometry, calibration, zeroing

class Lens(C.Structure):
    _fields_ = [(n, C.c_double) for n in ("f", "cx", "cy", "k1", "k2", "p1", "p2")]


class H33(C.Structure):
    _fields_ = [("m", C.c_double * 9)]


class Geom(C.Structure):
    _fields_ = [("lens", Lens), ("to_cam", H33), ("to_screen", H33), ("half_w", C.c_double),
                ("half_h", C.c_double)]


class CalibCfg(C.Structure):
    _fields_ = [("max_fit_rms_px", C.c_double), ("max_holdout_rms_px", C.c_double),
                ("gate_px", C.c_double), ("min_found", C.c_int), ("holdout_every", C.c_int),
                ("max_iter", C.c_int)]


class CalibReport(C.Structure):
    _fields_ = [("status", C.c_int), ("n_found", C.c_int), ("n_fit", C.c_int), ("n_holdout", C.c_int),
                ("iterations", C.c_int), ("fit_rms_px", C.c_double), ("holdout_rms_px", C.c_double),
                ("max_px", C.c_double)]

    def as_dict(self) -> dict:
        return {n: getattr(self, n) for n, _ in self._fields_}


class Zero(C.Structure):
    _fields_ = [("n", C.c_int), ("sx", C.c_double), ("sy", C.c_double), ("sxx", C.c_double),
                ("syy", C.c_double), ("valid", C.c_bool), ("off_x", C.c_double), ("off_y", C.c_double),
                ("spread_mm", C.c_double), ("shots", C.c_int)]


CAL_STATUS = {0: "ok", -1: "too_few", -2: "order", -3: "no_converge", -4: "residual"}
GEOM_STATUS = {0: "on_screen", 1: "off_screen", -1: "fail"}
D = C.POINTER(C.c_double)


def setup_geom(L: C.CDLL) -> None:
    L.at_lens_distort.argtypes = [C.POINTER(Lens), C.c_double, C.c_double, D, D]
    L.at_lens_undistort.argtypes = [C.POINTER(Lens), C.c_double, C.c_double, D, D]
    L.at_lens_undistort.restype = C.c_bool
    L.at_h33_apply.argtypes = [C.POINTER(H33), C.c_double, C.c_double, D, D]
    L.at_h33_apply.restype = C.c_bool
    L.at_h33_invert.argtypes = [C.POINTER(H33), C.POINTER(H33)]
    L.at_h33_invert.restype = C.c_bool
    L.at_h33_dlt.argtypes = [D, D, C.c_int, C.POINTER(H33)]
    L.at_geom_init.argtypes = [C.POINTER(Geom), C.POINTER(Lens), C.POINTER(H33), C.c_double, C.c_double]
    L.at_geom_to_screen.argtypes = [C.POINTER(Geom), C.c_double, C.c_double, D, D]
    L.at_geom_to_image.argtypes = [C.POINTER(Geom), C.c_double, C.c_double, D, D]
    L.at_geom_to_image.restype = C.c_bool
    L.at_calib_cfg_default.argtypes = [C.POINTER(CalibCfg)]
    L.at_calib_order.argtypes = [D, C.c_int, D, C.c_int, C.POINTER(Lens), C.c_double, C.POINTER(C.c_int)]
    L.at_calib_run.argtypes = [D, C.c_int, D, C.c_int, C.POINTER(Lens), C.POINTER(CalibCfg), C.c_double,
                               C.c_double, C.POINTER(Geom), C.POINTER(CalibReport)]
    L.at_zero_reset.argtypes = [C.POINTER(Zero)]
    L.at_zero_begin.argtypes = [C.POINTER(Zero)]
    L.at_zero_add.argtypes = [C.POINTER(Zero)] + [C.c_double] * 4
    L.at_zero_finish.argtypes = [C.POINTER(Zero), C.c_int, C.c_double]
    L.at_zero_apply.argtypes = [C.POINTER(Zero), D, D]


def _darr(a) -> C.Array:
    a = np.ascontiguousarray(np.asarray(a, dtype=np.float64).reshape(-1))
    return (C.c_double * len(a))(*a)


def lens_from(f: float, cx: float, cy: float, k=(0.0, 0.0, 0.0, 0.0)) -> Lens:
    return Lens(f, cx, cy, *k)


def calibrate(L: C.CDLL, img_pts, grid_mm, nominal: Lens, screen_w: float, screen_h: float,
              cfg: CalibCfg | None = None) -> tuple[Geom | None, dict]:
    """at_calib_run on detected dots (N x 2 px) and the grid (M x 2 mm). Returns (geometry or None, report)."""
    if cfg is None:
        cfg = CalibCfg()
        L.at_calib_cfg_default(C.byref(cfg))
    img, grid = np.asarray(img_pts), np.asarray(grid_mm)
    g, rep = Geom(), CalibReport()
    st = L.at_calib_run(_darr(img), len(img), _darr(grid), len(grid), C.byref(nominal), C.byref(cfg),
                        screen_w, screen_h, C.byref(g), C.byref(rep))
    r = rep.as_dict()
    r["status"] = CAL_STATUS[st]
    return (g if st == 0 else None), r


def to_screen(L: C.CDLL, g: Geom, u: float, v: float) -> tuple[float, float, str]:
    x, y = C.c_double(), C.c_double()
    st = L.at_geom_to_screen(C.byref(g), u, v, C.byref(x), C.byref(y))
    return x.value, y.value, GEOM_STATUS[st]


def to_image(L: C.CDLL, g: Geom, x: float, y: float) -> tuple[float, float]:
    u, v = C.c_double(), C.c_double()
    if not L.at_geom_to_image(C.byref(g), x, y, C.byref(u), C.byref(v)):
        raise ValueError("point maps to infinity")
    return u.value, v.value


def geom_dict(g: Geom) -> dict:
    """Plain values of a geometry (calib.json)."""
    return {"lens": {n: getattr(g.lens, n) for n, _ in Lens._fields_},
            "to_cam": list(g.to_cam.m), "to_screen": list(g.to_screen.m),
            "screen_w": 2 * g.half_w, "screen_h": 2 * g.half_h}


def geom_from_dict(L: C.CDLL, d: dict) -> Geom:
    g = Geom()
    lens = Lens(*[d["lens"][n] for n, _ in Lens._fields_])
    h = H33((C.c_double * 9)(*d["to_cam"]))
    if L.at_geom_init(C.byref(g), C.byref(lens), C.byref(h), d["screen_w"], d["screen_h"]) != 0:
        raise ValueError("bad geometry")
    return g
