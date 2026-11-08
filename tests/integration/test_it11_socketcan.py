"""IT11 SocketCAN smoke test: aim_host live on a virtual CAN interface (vcan).

The other integration tests replay the CAN traffic from log files, so the
live transport would otherwise run in no test. Here, in real time:
  - SYNC goes out on vcan every 10 ms, carrying the lane time of the moment
    it is sent (lane time = CLOCK_MONOTONIC + an offset);
  - each camera frame is written into aim_host's pipe 2.5 ms after its
    exposure ends, stamped with its exposure start on CLOCK_MONOTONIC (the
    tracker's clock);
  - one weapon aims at a fixed point and fires once; its SHOT_EVENT goes out
    1.2 ms after the break.
Checked: the HIT comes back on the bus, OK and on the spot; every received
frame carried a kernel timestamp and no SYNC was held out as late; frame
times from SYNC within P07's 100 us; TRK_STATUS arrives.

Needs an interface that is up (default vcan0, or AIMTRK_VCAN), for example
    sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
and is skipped without one.
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import select
import socket
import struct
import subprocess
import time

import numpy as np
import pytest

import lane as LN
import scenario as SC

IFACE = os.environ.get("AIMTRK_VCAN", "vcan0")
CAN_FRAME = struct.Struct("=IB3x8s")
K0, N_FRAMES, K_SHOT = 1000, 160, 1100  # exposures; the shot breaks in K_SHOT's window
AIM = (120.0, -80.0)  # screen mm
READOUT_US = 2500
LANE_START = K0 * SC.FRAME_US - 400_000  # SYNC starts 0.4 s before the first frame
FN_HIT, FN_TRK_STATUS = 0xA, 0xC
SYNC_ID = 0x080
SYNC_DATA = struct.Struct("<BI")  # protocol/tracker.dbc SYNC: isotp_pci (4), lane_time_us


def iface_up(name: str) -> bool:
    flags = pathlib.Path("/sys/class/net") / name / "flags"
    return hasattr(socket, "AF_CAN") and flags.exists() and int(flags.read_text(), 16) & 0x1 == 0x1


def mono_us() -> int:
    return time.monotonic_ns() // 1000


def send(sock: socket.socket, name: str, node: int, **signals) -> None:
    msg = SC.DBC.get_message_by_name(name)
    data = msg.encode(signals, strict=True)
    fid = msg.frame_id + (node - 1 if node > 0 else 0)
    sock.send(CAN_FRAME.pack(fid, len(data), data.ljust(8, b"\0")))


@pytest.mark.skipif(not iface_up(IFACE), reason=f"no CAN interface {IFACE} that is up (AIMTRK_VCAN)")
def test_socketcan_live(lib, calibration, tmp_path):
    cam = LN.camsim.Camera()
    rend = SC.RegionRenderer(cam, np.random.default_rng(11))
    frames = []
    for k in range(K0, K0 + N_FRAMES):  # the weapon (node 1, slot 0) flashes in even frames and its shot frame
        spots = []
        if k % 2 == 0 or k == K_SHOT:
            spots = [LN.camsim.Spot(xy_mm=AIM, total_dn=SC.nominal_total_dn(), weapon=1,
                                    kind="shot" if k == K_SHOT else "aim")]
        frames.append(rend.frame(k, spots)[0])

    run = tmp_path / "it11"
    run.mkdir()
    bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind((IFACE,))
    bus.setblocking(False)
    host = subprocess.Popen(
        [str(LN.build_host()), "--frames", "-", "--calib", str(calibration["path"]), "--weapon", "1:0",
         "--slots", "2", "--can", IFACE, "--can-log", str(run / "can_out.log"), "--aar", str(run / "aar.jsonl"),
         "--frame-log", str(run / "frames.jsonl"), "--readout-us", str(READOUT_US)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    received = []

    def drain() -> None:
        while select.select([bus], [], [], 0)[0]:
            fid, n, data = CAN_FRAME.unpack(bus.recv(CAN_FRAME.size))
            received.append((fid, data[:n]))

    try:
        time.sleep(0.2)  # aim_host is up and bound
        base = mono_us() + 100_000  # local time of lane time LANE_START
        events = []  # (local time, kind, arg)
        for t in range(LANE_START, (K0 + N_FRAMES + 5) * SC.FRAME_US, SC.SYNC_PERIOD_US):
            events.append((base + t - LANE_START, 0, None))
        for i, k in enumerate(range(K0, K0 + N_FRAMES)):
            t0 = base + k * SC.FRAME_US + SC.EXPOSURE_START_US - LANE_START
            events.append((t0 + LN.camsim.EXPOSURE_US + READOUT_US, 1, (i, t0)))
        t_break = K_SHOT * SC.FRAME_US + 1000
        events.append((base + t_break + 1200 - LANE_START, 2, t_break))
        events.sort(key=lambda e: e[0])

        late_us = 0
        for t, kind, arg in events:
            while (now := mono_us()) < t:
                time.sleep(min(t - now, 1000) / 1e6)
            late_us = max(late_us, now - t)
            if kind == 0:  # SYNC: the lane time of this moment (packed by hand: no encoding between clock and send)
                lane_us = (mono_us() - base + LANE_START) % (1 << 32)
                bus.send(CAN_FRAME.pack(SYNC_ID, 5, SYNC_DATA.pack(4, lane_us).ljust(8, b"\0")))
            elif kind == 1:
                i, t0 = arg
                LN.camsim.write_frame(host.stdin, frames[i], i, t_local_us=t0)
                host.stdin.flush()
            else:
                send(bus, "SHOT_EVENT_N1", 1, shot_no=1, t_us=arg % (1 << 32), mode=1, chambered=1, rounds_left=9)
            drain()
        out, err = host.communicate(timeout=10)  # closes the pipe: aim_host ends
    finally:
        if host.poll() is None:
            host.kill()
    time.sleep(0.05)
    drain()
    bus.close()
    assert host.returncode == 0, err.decode()
    (run / "summary.json").write_text(out.decode())
    res = LN.read_outputs(run)
    summ = res["summary"]

    hits_on_bus = [SC.DBC.get_message_by_name("HIT_N1").decode(d, decode_choices=False)
                   for fid, d in received if fid >> 7 == FN_HIT and fid & 0x7F == 1]
    status_on_bus = [d for fid, d in received if fid >> 7 == FN_TRK_STATUS]
    result = {"driver_late_us_max": late_us, "hits_on_bus": hits_on_bus, "status_on_bus": len(status_on_bus),
              "aims_logged": len(res["aims"]), "summary": {k: summ[k] for k in (
                  "frames", "camera_dropped", "frame_time_residual_max_us", "drift_ppm", "sync",
                  "can_tx_dropped", "can_user_stamps", "can_rx_delay_max_us")}}
    LN.write_result("it11_socketcan.json", json.loads(json.dumps(result, default=float)))

    assert len(hits_on_bus) == 1, hits_on_bus
    h = hits_on_bus[0]
    assert h["result"] == 0 and h["shot_no"] == 1
    assert math.hypot(h["x_mm"] - AIM[0], h["y_mm"] - AIM[1]) <= 1.0, h
    assert h["latency"] <= 25.0  # P02
    assert summ["frames"] == N_FRAMES and summ["camera_dropped"] == 0
    assert summ["can_user_stamps"] == 0, "every frame carried a kernel timestamp"
    assert summ["sync"]["rejected"] == 0, "no SYNC was stamped late"
    assert summ["can_tx_dropped"] == 0
    assert summ["frame_time_residual_max_us"] <= 100  # P07
    assert len(res["aims"]) >= 0.9 * N_FRAMES / 2 - 4
    assert status_on_bus, "TRK_STATUS on the bus"
