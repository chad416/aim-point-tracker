"""IT10 co-simulation, tracker side (SPEC §9.3 IT10).

The weapon firmware runs in Renode (tools/cosim/s1_lane_driver.py, on its
sim/lane.resc) and records every emitter gate edge and every CAN frame the two
weapons send. This module turns that recording into what aim_host sees and runs
it:

  can_in.log  the lane's SYNC frames as injected, and the weapons' own frames
              (SHOT_EVENT, SHOT_DETAIL, STATUS, ...) at the virtual time they
              were sent, all on the tracker's clock (offset and drift against
              lane time, as in the scenarios)
  frames      one camera frame per exposure: each pulse the firmware actually gave
              (gate on -> off, in lane time) is rendered where that weapon aims
              at that moment, with the part of it that falls inside the 1 ms
              exposure; nothing is rendered for a weapon that did not flash

So whether a HIT has a position depends on the firmware putting its pulses into the
right exposures, on its own lane-time estimate, and on its SHOT_EVENT carrying
the right break time.

Run:  python tools/cosim/cosim.py [--events FILE] [--renode] [--out DIR]
  --renode  run Renode first (needs the weapon firmware's ELF and repository, see firmware_paths()).
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import pathlib
import subprocess
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "tools" / "lane"))

import camsim  # noqa: E402
import runner  # noqa: E402
import scenario as SC  # noqa: E402

FN_SHOT_EVENT = 2
FN_SYNC = 1
WEAPONS = {1: 0, 2: 1}  # node -> slot (the driver sets STROBE_SLOT the same way)
N_SLOTS = 2


def aim(node: int, t_s: float) -> tuple[float, float]:
    """Where each shooter aims (bore, screen mm) at lane time t_s: slow sweeps with a little wobble."""
    if node == 1:
        return (-700.0 + 400.0 * math.sin(0.9 * t_s) + 2 * math.sin(11 * t_s),
                150.0 * math.sin(1.3 * t_s) + 2 * math.cos(8 * t_s))
    return (650.0 + 350.0 * math.sin(0.7 * t_s + 1.0) + 2 * math.sin(9 * t_s),
            -200.0 + 180.0 * math.sin(1.1 * t_s) + 2 * math.cos(7 * t_s))


def firmware_paths() -> tuple[str, str]:
    """The weapon firmware (trainer-weapon-firmware-twin) for the Renode run, from the environment:
      S1_ELF  its zephyr.elf     (default $AIMTRK_WS/build/app/zephyr/zephyr.elf)
      S1_SIM  its sim/ directory (default $S1_SRC/sim; S1_SRC, its checkout, default ~/trainer-src)
    AIMTRK_WS is the shared west workspace (default ~/trainer-ws), as in scripts/_env.sh."""
    ws = os.environ.get("AIMTRK_WS", os.path.expanduser("~/trainer-ws"))
    src = os.environ.get("S1_SRC", os.path.expanduser("~/trainer-src"))
    return (os.environ.get("S1_ELF", os.path.join(ws, "build", "app", "zephyr", "zephyr.elf")),
            os.environ.get("S1_SIM", os.path.join(src, "sim")))


def run_renode(out: pathlib.Path) -> None:
    """Run the weapon firmware's lane in Renode with the driver. Paths (inside Linux/WSL, no
    spaces: Renode's monitor cannot take them): firmware_paths(), and COSIM_DRIVER."""
    out = out.resolve()  # the driver runs in Renode's own directory: a relative path would land there
    elf, sim = firmware_paths()
    drv = pathlib.Path(os.environ.get("COSIM_DRIVER", str(pathlib.Path(__file__).with_name("s1_lane_driver.py"))))
    if " " in str(drv):  # the monitor cannot take the path: use a copy next to the output
        (out.parent / drv.name).write_bytes(drv.read_bytes())
        drv = out.parent / drv.name
    script = out.with_suffix(".resc")
    # A 10 us quantum: the edge times are the machines' virtual time, which the default quantum
    # makes several hundred microseconds coarse (seen as pulses up to 460 us late).
    script.write_text(f"$elf=@{elf}\ninclude @{sim}/lane.resc\nemulation SetGlobalQuantum \"0.00001\"\n"
                      f"include @{drv}\nquit\n")
    env = dict(os.environ, COSIM_OUT=str(out))
    log = out.with_suffix(".renode.log")
    with open(log, "w") as f:
        subprocess.run(["renode", "--disable-gui", "--console", "-e", f"include @{script}"],
                       env=env, stdout=f, stderr=subprocess.STDOUT, check=True, timeout=1800)
    if not out.exists():
        raise RuntimeError(f"Renode wrote no events, see {log}")


@dataclasses.dataclass
class Recording:
    lane0_us: int
    edges: dict[int, list[tuple[int, int]]]      # node -> [(lane_us, state)]
    sent: list[tuple[int, int, int, bytes]]       # (lane_us, node, id, data)
    injected: list[tuple[int, int, bytes]]        # (lane_us, id, data)


def parse(path: pathlib.Path) -> Recording:
    lane0, edges, sent, injected = 0, {n: [] for n in WEAPONS}, [], []
    rows = [ln.split() for ln in path.read_text().splitlines() if ln.strip()]
    for r in rows:
        if r[0] == "L":
            lane0 = int(r[1])
    for r in rows:
        if r[0] == "E":
            edges[int(r[1])].append((int(r[2]) + lane0, int(r[3])))
        elif r[0] == "T":
            sent.append((int(r[2]) + lane0, int(r[1]), int(r[3], 16), bytes.fromhex(r[4] if len(r) > 4 else "")))
        elif r[0] == "I":
            injected.append((int(r[1]) + lane0, int(r[2], 16), bytes.fromhex(r[3])))
    return Recording(lane0, edges, sent, injected)


def pulses(rec: Recording) -> list[dict]:
    """Emitter pulses (gate on -> off) of both weapons, in lane time."""
    out = []
    for node, ev in rec.edges.items():
        on = None
        for t, s in ev:
            if s == 1 and on is None:
                on = t
            elif s == 0 and on is not None:
                out.append({"node": node, "start_us": on, "end_us": t})
                on = None
    return sorted(out, key=lambda p: p["start_us"])


def shot_events(rec: Recording) -> list[dict]:
    """The weapons' SHOT_EVENT frames: shot number and the break time it carries (lane time, 32 bits)."""
    out = []
    for t, node, fid, data in rec.sent:
        if fid >> 7 == FN_SHOT_EVENT and len(data) >= 6:
            shot_no = data[0] | data[1] << 8
            t32 = int.from_bytes(data[2:6], "little")
            t_break = t32 + ((t - t32) // (1 << 32) + (1 if (t - t32) % (1 << 32) > (1 << 31) else 0)) * (1 << 32)
            out.append({"node": fid & 0x7F, "shot_no": shot_no, "t_break_us": t_break, "sent_us": t,
                        "k": SC.shot_frame(t_break)})
    return out


def build_and_run(rec: Recording, run_dir: pathlib.Path, calib: pathlib.Path, cam=None, seed: int = 10) -> dict:
    """Write can_in.log and the frames from the recording, run aim_host, and return the truth and outputs."""
    run_dir.mkdir(parents=True, exist_ok=True)
    cam = cam or camsim.Camera()
    rng = np.random.default_rng(seed)
    rend = SC.RegionRenderer(cam, rng, None)
    clk = SC.Clock()
    total_dn = SC.nominal_total_dn()

    can = [(t, fid, data) for t, fid, data in rec.injected if fid >> 7 == FN_SYNC]
    can += [(t, fid, data) for t, node, fid, data in rec.sent]
    can.sort(key=lambda c: c[0])
    with open(run_dir / "can_in.log", "w") as f:
        for t, fid, data in can:
            tl = clk.local(t)
            f.write(f"({tl // 1_000_000}.{tl % 1_000_000:06d}) can0 {fid:03X}#{data.hex().upper()}\n")

    ps = pulses(rec)
    shots = shot_events(rec)
    shot_k = {(s["node"], s["k"]): s for s in shots}
    sync_t = [t for t, fid, _ in rec.injected if fid >> 7 == FN_SYNC]
    k_first = -((-(sync_t[0] + 200_000 - SC.EXPOSURE_START_US)) // SC.FRAME_US)
    k_last = (sync_t[-1] - SC.EXPOSURE_START_US) // SC.FRAME_US
    by_k: dict[int, list[dict]] = {}
    for p in ps:
        for k in range(p["start_us"] // SC.FRAME_US - 1, p["end_us"] // SC.FRAME_US + 1):
            e0 = k * SC.FRAME_US + SC.EXPOSURE_START_US
            if p["start_us"] < e0 + camsim.EXPOSURE_US and p["end_us"] > e0:
                by_k.setdefault(k, []).append(p)

    truth_pulses = []
    frames_path = run_dir / "frames.bin"
    with open(frames_path, "wb") as f:
        for index, k in enumerate(range(k_first, k_last)):
            spots, meta = [], []
            for p in by_k.get(k, []):
                s0, s1 = p["start_us"], p["end_us"]
                p0 = camsim.emitter_point(aim(p["node"], s0 * 1e-6))
                p1 = camsim.emitter_point(aim(p["node"], s1 * 1e-6))
                vel = (p1 - p0) / max((s1 - s0) * 1e-6, 1e-6)
                kind = "shot" if (p["node"], k) in shot_k else "aim"
                # total_dn is the signal of a nominal 0.5 ms pulse; a longer or cut pulse scales with it.
                spots.append(camsim.Spot(xy_mm=(float(p0[0]), float(p0[1])),
                                         total_dn=total_dn * (s1 - s0) / SC.PULSE_US, t_us=(s0, s1),
                                         vel_mm_s=(float(vel[0]), float(vel[1])), weapon=p["node"], kind=kind))
                meta.append({"node": p["node"], "k": k, "kind": kind, "start_us": s0, "end_us": s1,
                             "err_us": s0 - (k * SC.FRAME_US + SC.PULSE_START_US)})
            img, recs = rend.frame(k, spots)
            for m, r in zip(meta, recs):
                m.update(x_mm=r["x_mm"], y_mm=r["y_mm"], rendered=r["rendered"], hidden=r.get("hidden", False))
                truth_pulses.append(m)
            camsim.write_frame(f, img, index, clk.local(k * SC.FRAME_US + SC.EXPOSURE_START_US))

    for s in shots:
        p = [q for q in truth_pulses if q["node"] == s["node"] and q["k"] == s["k"]]
        s["pulse"] = p[0] if p else None
    truth = {"k_first": k_first, "k_last": k_last, "shots": shots, "pulses": truth_pulses,
             "edges": {n: len(e) for n, e in rec.edges.items()}, "can_sent": len(rec.sent)}
    (run_dir / "truth.json").write_text(json.dumps(truth) + "\n")

    host = runner.build_host()
    args = [str(host), "--frames", str(frames_path), "--calib", str(calib),
            "--can-in", str(run_dir / "can_in.log"), "--can-out", str(run_dir / "can_out.log"),
            "--aar", str(run_dir / "aar.jsonl"), "--frame-log", str(run_dir / "frames.jsonl"),
            "--slots", str(N_SLOTS)]
    for node, slot in WEAPONS.items():
        args += ["--weapon", f"{node}:{slot}"]
    with open(run_dir / "summary.json", "w") as out_f, open(run_dir / "stderr.txt", "w") as err_f:
        rc = subprocess.run(args, stdout=out_f, stderr=err_f).returncode
    if rc != 0:
        raise RuntimeError(f"aim_host failed ({rc}): {(run_dir / 'stderr.txt').read_text()}")
    return {"truth": truth, "out": runner.read_outputs(run_dir)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--events", type=pathlib.Path, default=ROOT / "tests" / "data" / "s1_v0.6.2_lane.txt")
    ap.add_argument("--renode", action="store_true", help="run the weapon firmware in Renode first")
    ap.add_argument("--calib", type=pathlib.Path, default=ROOT / "results" / "calib.json")
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "results" / "cosim_run")
    a = ap.parse_args()
    if a.renode:
        a.events.parent.mkdir(parents=True, exist_ok=True)
        run_renode(a.events)
    r = build_and_run(parse(a.events), a.out, a.calib)
    print(json.dumps({"shots": len(r["truth"]["shots"]), "hits": len(r["out"]["hits"]),
                      "aims": len(r["out"]["aims"])}))


if __name__ == "__main__":
    main()
