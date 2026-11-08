"""Shared set-up for the integration tests (SPEC §9.3): the simulated lane, the
C library, and a shot pipeline (render -> detect -> map to the screen).

Until aim_host exists (week 3), the tests drive the C library through ctypes
on frames from the camera simulator; the pipeline is the same.
"""
from __future__ import annotations

import json
import math
import pathlib
import subprocess
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


HOST = ROOT / "build" / "aim_host"
_built = False


def build_host() -> pathlib.Path:
    """make -C host (once per session)."""
    global _built
    if not _built:
        subprocess.run(["make", "-s", "-C", str(ROOT / "host")], check=True)
        _built = True
    return HOST


def _decode(db, line: str):
    head, frame = line.split()[0], line.split()[2]
    t_us = int(round(float(head.strip("()")) * 1e6))
    fid_s, data_s = frame.split("#")
    fid, data = int(fid_s, 16), bytes.fromhex(data_s)
    fn, node = fid >> 7, fid & 0x7F
    names = {0xA: "HIT_N1", 0xB: "AIM_N1", 0xC: "TRK_STATUS"}
    if fn not in names:
        return None
    msg = db.get_message_by_name(names[fn])
    d = dict(msg.decode(data, decode_choices=False))
    d.update(kind=names[fn].split("_N1")[0], node=node, t_local_us=t_us)
    return d


def run_scenario(scn, run_dir: pathlib.Path, calib: pathlib.Path, cam=None, *extra: str) -> tuple[dict, dict]:
    """Generate the scenario straight into aim_host (frames through a pipe). Returns (truth, outputs)."""
    import scenario
    build_host()
    run_dir.mkdir(parents=True, exist_ok=True)
    weapons = [(w.node, w.slot) for w in scn.weapons]
    args = [str(HOST), "--frames", "-", "--calib", str(calib),
            "--can-in", str(run_dir / "can_in.log"), "--can-out", str(run_dir / "can_out.log"),
            "--aar", str(run_dir / "aar.jsonl"), "--frame-log", str(run_dir / "frames.jsonl"),
            "--slots", str(scn.n_slots)]
    for node, slot in weapons:
        args += ["--weapon", f"{node}:{slot}"]
    args += list(extra)
    out_f = open(run_dir / "summary.json", "w")
    err_f = open(run_dir / "stderr.txt", "w")
    proc = {}

    def opener():
        proc["p"] = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=out_f, stderr=err_f)
        return proc["p"].stdin

    truth = scenario.generate(scn, run_dir, cam, frames_opener=opener)
    rc = proc["p"].wait()
    out_f.close()
    err_f.close()
    if rc != 0:
        raise RuntimeError(f"aim_host failed ({rc}): {(run_dir / 'stderr.txt').read_text()}")
    return truth, _outputs(run_dir)


def _outputs(run_dir: pathlib.Path) -> dict:
    import scenario
    out = [_decode(scenario.DBC, ln) for ln in (run_dir / "can_out.log").read_text().splitlines() if ln.strip()]
    out = [o for o in out if o is not None]
    lines = lambda name: [json.loads(ln) for ln in (run_dir / name).read_text().splitlines() if ln.strip()]  # noqa: E731
    return {
        "summary": json.loads((run_dir / "summary.json").read_text().strip().splitlines()[-1]),
        "hits": [o for o in out if o["kind"] == "HIT"], "aims": [o for o in out if o["kind"] == "AIM"],
        "status": [o for o in out if o["kind"] == "TRK_STATUS"],
        "aar": lines("aar.jsonl"), "frames": lines("frames.jsonl"),
    }


def write_result(name: str, data: dict) -> pathlib.Path:
    dst = RESULTS / name
    dst.write_text(json.dumps(data, indent=1) + "\n")
    return dst
