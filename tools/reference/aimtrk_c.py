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
                ("frozen_cells", C.c_uint32), ("overflow", C.c_uint8), ("learning", C.c_bool)]


class CWin(C.Structure):
    _fields_ = [("px", C.POINTER(C.c_uint8)), ("stride", C.c_size_t), ("x0", C.c_int),
                ("y0", C.c_int), ("w", C.c_int), ("h", C.c_int), ("mask", C.POINTER(C.c_uint8)),
                ("bg", C.POINTER(C.c_float)), ("bg_const", C.c_float)]


def build(lib: pathlib.Path = LIB) -> None:
    lib.parent.mkdir(parents=True, exist_ok=True)
    src = sorted(str(p) for p in (ROOT / "aimtrk" / "src").glob("*.c"))
    subprocess.run(["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Wconversion", "-Werror",
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
