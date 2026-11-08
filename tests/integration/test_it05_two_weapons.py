"""IT05 two weapons (SPEC F09, P04), through aim_host; and the bus load (SPEC §7).

Weapon 1 (slot 0) and weapon 2 (slot 1), each with its own sway.
  A  staggered shots, at least one frame apart          P04: 100 % attributed
  B  both fire into the same frame, aims 0.5 m apart    P04: >= 99 % attributed
  C  both fire into the same frame at nearly the same   AMBIGUOUS, never a wrong
     point (8 mm apart)                                 attribution
A HIT is attributed correctly when it is OK and within 2 mm of its own
weapon's true spot; wrong when it lands on the other weapon's spot.
"""
from __future__ import annotations

import math

import lane as LN
import scenario as SC

T0 = 1000.0
A_END, B_END, C_END = T0 + 6.0, T0 + 12.0, T0 + 16.5
BITRATE = 500_000


def aim1(t: float) -> tuple[float, float]:
    sway = (5 * math.sin(2 * math.pi * 1.2 * t), 4 * math.cos(2 * math.pi * 0.9 * t))
    if t < B_END:
        return -250 + 200 * math.sin(0.4 * (t - T0)) + sway[0], 100 + sway[1]
    return 0.0 + 0.3 * sway[0], 0.0 + 0.3 * sway[1]


def aim2(t: float) -> tuple[float, float]:
    sway = (4 * math.sin(2 * math.pi * 1.5 * t + 2), 5 * math.cos(2 * math.pi * 1.1 * t + 1))
    if t < B_END:
        return 250 + 200 * math.cos(0.3 * (t - T0)) + sway[0], -150 + sway[1]
    return 8.0 + 0.3 * sway[0], 0.0 + 0.3 * sway[1]


def frame_mid(k: int) -> float:
    return (k * SC.FRAME_US + 1500) * 1e-6  # well inside the frame's break window


def shots() -> tuple[list[float], list[float]]:
    b1, b2 = [], []
    k0 = int(T0 * 100) + 50
    for i in range(30):  # A: alternate, 10 frames apart
        b1.append(frame_mid(k0 + 20 * i))
        b2.append(frame_mid(k0 + 20 * i + 10))
    k0 = int(A_END * 100) + 60  # after A's last shot (k0_A + 590)
    for i in range(50):  # B: same frame, 10 frames apart
        b1.append(frame_mid(k0 + 10 * i))
        b2.append(frame_mid(k0 + 10 * i) + 0.001)  # 1 ms later, same shot frame
    k0 = int(B_END * 100) + 100  # after the tracks have moved to the shared point
    for i in range(30):  # C: same frame, nearly the same point
        b1.append(frame_mid(k0 + 10 * i))
        b2.append(frame_mid(k0 + 10 * i) + 0.001)
    return b1, b2


def frame_bits(dlc: int) -> float:
    """Standard data frame with worst-case bit stuffing, plus the 3-bit intermission."""
    return 47 + 8 * dlc + math.floor((34 + 8 * dlc - 1) / 4) + 3


def test_two_weapons(lib, cam, calibration, tmp_path):
    b1, b2 = shots()
    w1 = SC.Weapon(node=1, slot=0, aim=aim1, breaks_s=b1)
    w2 = SC.Weapon(node=2, slot=1, aim=aim2, breaks_s=b2)
    scn = SC.Scenario(weapons=[w1, w2], t0_s=T0, duration_s=C_END - T0 + 0.3, seed=51)
    truth, out = LN.run_scenario(scn, tmp_path / "it05", calibration["path"], cam)

    hits = {(h["node"], h["shot_no"]): h for h in out["hits"]}
    by_frame = {}
    for s in truth["shots"]:
        by_frame.setdefault(s["k"], []).append(s)
    seg = {"A": [0, 0, 0, 0], "B": [0, 0, 0, 0], "C": [0, 0, 0, 0]}  # shots, correct, wrong, ambiguous
    failures = []
    for s in truth["shots"]:
        i = s["shot_no"] - 1  # shots are numbered in the order shots() builds them: 30 A, 50 B, 30 C
        name = "A" if i < 30 else ("B" if i < 80 else "C")
        h = hits[(s["node"], s["shot_no"])]
        other = [o for o in by_frame[s["k"]] if o["node"] != s["node"]]
        seg[name][0] += 1
        if h["result"] == 3:
            seg[name][3] += 1
            if name != "C":
                failures.append({"segment": name, "node": s["node"], "shot_no": s["shot_no"], "k": s["k"],
                                 "result": 3})
            continue
        d_own = math.hypot(h["x_mm"] - s["x_mm"], h["y_mm"] - s["y_mm"])
        d_other = min((math.hypot(h["x_mm"] - o["x_mm"], h["y_mm"] - o["y_mm"]) for o in other), default=1e9)
        if h["result"] == 0 and d_own <= 2.0:
            seg[name][1] += 1
            continue
        if d_other < d_own:
            seg[name][2] += 1
        failures.append({"segment": name, "node": s["node"], "shot_no": s["shot_no"], "k": s["k"],
                         "result": h["result"], "d_own_mm": round(d_own, 2), "d_other_mm": round(d_other, 2),
                         "truth": [round(s["x_mm"], 1), round(s["y_mm"], 1)],
                         "hit": [h["x_mm"], h["y_mm"]]})

    # Bus load: the tracker's frames as measured, plus the weapons' frames at their documented rates.
    dur = scn.duration_s
    out_counts = {"HIT": len(out["hits"]), "AIM": len(out["aims"]), "TRK_STATUS": len(out["status"])}
    tracker_bps = (len(out["hits"]) * frame_bits(8) + len(out["aims"]) * frame_bits(6)
                   + len(out["status"]) * frame_bits(7)) / dur
    # Worst case (SPEC 7): two weapons at 900 rpm: SHOT_EVENT + SHOT_DETAIL per shot, STATUS 10 Hz each,
    # SYNC 100 Hz; the tracker: HIT per shot, AIM at 100 Hz total, TRK_STATUS 1 Hz.
    shots_per_s = 2 * 900 / 60
    worst = {"SYNC": 100, "SHOT_EVENT+DETAIL": 2 * shots_per_s, "S1_STATUS": 20, "HIT": shots_per_s,
             "AIM": 100, "TRK_STATUS": 1}
    worst_bps = (100 * frame_bits(5) + 2 * shots_per_s * frame_bits(8) + 20 * frame_bits(8)
                 + shots_per_s * frame_bits(8) + 100 * frame_bits(6) + 1 * frame_bits(7))
    res = {
        "segments": {k: dict(zip(("shots", "correct", "wrong", "ambiguous"), v)) for k, v in seg.items()},
        "tracker_frames": out_counts, "tracker_bus_load_pct": 100 * tracker_bps / BITRATE,
        "worst_case_frames_per_s": worst, "worst_case_frames_per_s_total": sum(worst.values()),
        "worst_case_bus_load_pct": 100 * worst_bps / BITRATE,
        "match": out["summary"]["match"], "failures": failures,
    }
    LN.write_result("it05_two_weapons.json", res)
    a, b, c = seg["A"], seg["B"], seg["C"]
    assert a[1] == a[0], "P04: staggered shots all attributed"
    assert b[1] >= 0.99 * b[0], "P04: same-frame shots >= 99 %"
    assert a[2] == b[2] == c[2] == 0, "never a wrong attribution"
    assert c[3] >= 0.9 * c[0], "nearly the same point: AMBIGUOUS"
    assert res["worst_case_bus_load_pct"] < 10.0
