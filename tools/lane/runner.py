"""Run a lane scenario through aim_host and read what it produced (SPEC §9.3, §10).

Used by the integration tests (tests/integration) and the demo (demo_session.py).
"""
from __future__ import annotations

import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))

HOST = ROOT / "build" / "aim_host"
_built = False


def build_host() -> pathlib.Path:
    """make -C host (once per session)."""
    global _built
    if not _built:
        subprocess.run(["make", "-s", "-C", str(ROOT / "host")], check=True)
        _built = True
    return HOST


def decode(db, line: str):
    """One candump -L line of the tracker's output, decoded with the DBC (None: not a tracker frame)."""
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
    return truth, read_outputs(run_dir)


def read_outputs(run_dir: pathlib.Path) -> dict:
    import scenario
    out = [decode(scenario.DBC, ln) for ln in (run_dir / "can_out.log").read_text().splitlines() if ln.strip()]
    out = [o for o in out if o is not None]
    lines = lambda name: [json.loads(ln) for ln in (run_dir / name).read_text().splitlines() if ln.strip()]  # noqa: E731
    return {
        "summary": json.loads((run_dir / "summary.json").read_text().strip().splitlines()[-1]),
        "hits": [o for o in out if o["kind"] == "HIT"], "aims": [o for o in out if o["kind"] == "AIM"],
        "status": [o for o in out if o["kind"] == "TRK_STATUS"],
        "aar": lines("aar.jsonl"), "frames": lines("frames.jsonl"),
    }


