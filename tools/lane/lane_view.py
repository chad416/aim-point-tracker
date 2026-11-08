#!/usr/bin/env python3
"""Lane view (SPEC §10): what the instructor sees, drawn from the tracker's own output.

Reads a session (aim_host's CAN log and after-action records, plus the
session's targets) and draws, at any moment of the session:
  - the screen with the targets, each weapon's aim trace over the last
    second and its current aim point, and every hit so far with its shot
    number (from HIT and AIM frames, decoded with protocol/tracker.dbc);
  - the after-action panel of the latest shot: the aim trace from 1.0 s
    before the break to 0.5 s after it, and the weapon's trigger data
    (S1's SHOT_DETAIL) merged into the record by the tracker.

Usage:
  lane_view.py SESSION_DIR --png out.png [--at SECONDS]      one picture
  lane_view.py SESSION_DIR --frames DIR [--fps 10]            a picture per frame (for the GIF/video)
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "tools" / "lane"))

import runner  # noqa: E402

WEAPON = {1: "#2a78d6", 2: "#eb6834", 3: "#1baf7a", 4: "#eda100"}  # categorical slots 1-4
INK, INK2, MUTED, GRID, SURFACE, SCREEN = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#fcfcfb", "#f2f1ec"
RESULT = {0: "OK", 1: "NO SPOT", 2: "OFF SCREEN", 3: "AMBIGUOUS", 4: "SATURATED", 5: "DROPPED"}
FAULTS = ("no calibration", "calibration old", "frames dropped", "detect overflow", "slow", "no sync")


def fault_text(bits: int) -> str:
    names = [n for i, n in enumerate(FAULTS) if bits & (1 << i)]
    return ", ".join(names) if names else "no faults"


class Session:
    def __init__(self, d: pathlib.Path):
        import scenario
        meta = json.loads((d / "session.json").read_text())
        self.targets = meta["targets"]
        self.radius = meta["target_radius_mm"]
        self.screen = meta["screen_mm"]
        out = [runner.decode(scenario.DBC, ln) for ln in (d / "can_out.log").read_text().splitlines() if ln.strip()]
        out = [o for o in out if o is not None]
        self.t0 = min(o["t_local_us"] for o in out)
        self.aims = [o for o in out if o["kind"] == "AIM"]
        self.hits = [o for o in out if o["kind"] == "HIT"]
        self.status = [o for o in out if o["kind"] == "TRK_STATUS"]
        self.records = {(r["node"], r["shot_no"]): r for r in
                        (json.loads(ln) for ln in (d / "aar.jsonl").read_text().splitlines() if ln.strip())}
        self.end = (max(o["t_local_us"] for o in out) - self.t0) * 1e-6

    def t(self, o) -> float:
        return (o["t_local_us"] - self.t0) * 1e-6


def style() -> None:
    plt.rcParams.update({
        "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2,
        "ytick.color": INK2, "axes.titlecolor": INK, "axes.titleweight": "bold", "axes.titlelocation": "left",
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "legend.frameon": False,
    })


def draw(s: Session, at: float, path: pathlib.Path) -> None:
    fig = plt.figure(figsize=(12.8, 5.6), dpi=100)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.75, 1], height_ratios=[1.25, 1], wspace=0.12, hspace=0.38,
                          left=0.03, right=0.98, top=0.9, bottom=0.08)
    ax = fig.add_subplot(gs[:, 0])
    w, h = s.screen
    ax.add_patch(Rectangle((-w / 2, -h / 2), w, h, facecolor=SCREEN, edgecolor=MUTED, lw=1))
    for name, (x, y) in s.targets.items():
        for r, a in ((s.radius, 0.35), (s.radius * 0.6, 0.55), (s.radius * 0.25, 0.9)):
            ax.add_patch(Circle((x, y), r, facecolor="none", edgecolor=INK2, lw=1, alpha=a))
        ax.text(x, y + s.radius + 25, name, color=INK2, ha="center", fontsize=9)
    for node, color in WEAPON.items():
        pts = [(s.t(a), a["x_mm"], a["y_mm"]) for a in s.aims if a["node"] == node and at - 1.0 <= s.t(a) <= at]
        if pts:
            p = np.array(pts)
            ax.plot(p[:, 1], p[:, 2], color=color, lw=1.5, alpha=0.8)
            ax.plot(p[-1, 1], p[-1, 2], "o", ms=9, mfc="none", mec=color, mew=2)
    shown = [hh for hh in s.hits if s.t(hh) <= at]
    # Hits on one target are a few mm apart: their labels fan out to the right of it, with leader lines.
    groups: dict[str, list] = {}
    for hh in shown:
        if hh["result"] in (0, 2, 4):
            near = min(s.targets, key=lambda n: np.hypot(hh["x_mm"] - s.targets[n][0], hh["y_mm"] - s.targets[n][1]))
            groups.setdefault(near, []).append(hh)
    for name, hs in groups.items():
        tx, ty = s.targets[name]
        for i, hh in enumerate(hs):
            c = WEAPON.get(hh["node"], INK)
            ax.plot(hh["x_mm"], hh["y_mm"], "X", ms=8, color=c, mec=SURFACE, mew=0.8, zorder=3)
            ang = np.radians(50 - 25 * i)
            lx, ly = tx + (s.radius + 70) * np.cos(ang), ty + (s.radius + 70) * np.sin(ang)
            ax.annotate(f"{hh['node']}.{hh['shot_no']}", (hh["x_mm"], hh["y_mm"]), xytext=(lx, ly),
                        textcoords="data", fontsize=7.5, color=INK2, va="center",
                        arrowprops={"arrowstyle": "-", "color": MUTED, "lw": 0.6, "shrinkA": 0, "shrinkB": 4})
    ax.set_xlim(-w / 2 - 60, w / 2 + 60)
    ax.set_ylim(-h / 2 - 60, h / 2 + 60)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    legend = [plt.Line2D([], [], color=c, lw=2, label=f"weapon {n}") for n, c in WEAPON.items()
              if any(a["node"] == n for a in s.aims)]
    ax.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.0, 0.0), ncol=len(legend), fontsize=8)
    ax.set_title(f"Lane screen, t = {at:5.1f} s   (aim over the last second, hits with weapon.shot)", fontsize=10)

    # After-action panel: the latest shot whose record is complete (0.5 s after its HIT).
    done = [hh for hh in shown if s.t(hh) + 0.55 <= at and (hh["node"], hh["shot_no"]) in s.records]
    at_ax = fig.add_subplot(gs[0, 1])
    tx_ax = fig.add_subplot(gs[1, 1])
    tx_ax.axis("off")
    if done:
        hh = done[-1]
        r = s.records[(hh["node"], hh["shot_no"])]
        tr = np.array(r["trace"]) if r["trace"] else np.zeros((0, 3))
        hx, hy = r["hit"]["x_mm"], r["hit"]["y_mm"]
        c = WEAPON.get(r["node"], INK)
        if len(tr):
            at_ax.plot(tr[:, 0], tr[:, 1] - hx, color=c, lw=1.8, label="horizontal")
            at_ax.plot(tr[:, 0], tr[:, 2] - hy, color=c, lw=1.8, ls=(0, (4, 2)), label="vertical")
        at_ax.axvline(0, color=MUTED, lw=1, ls=(0, (2, 2)))
        at_ax.set_xlim(-1000, 500)
        at_ax.set_xlabel("ms from the trigger break")
        at_ax.set_ylabel("aim − hit, mm")
        at_ax.grid(color=GRID, lw=0.6)
        at_ax.legend(loc="upper left", fontsize=8)
        at_ax.set_title(f"After-action: weapon {r['node']}, shot {r['shot_no']}", fontsize=10)
        d = r.get("detail") or {}
        rows = [("result", f"{r['hit']['result']}, latency {r['hit']['latency_ms']:.1f} ms"),
                ("hit", f"({hx:+.1f}, {hy:+.1f}) mm, SNR {r['hit']['quality']}"),
                ("take-up / squeeze", f"{d.get('takeup_ms', '-')} / {d.get('squeeze_ms', '-')} ms"),
                ("jerk", f"{d.get('jerk', '-')} ‰/ms"), ("muzzle rate", f"{d.get('muzzle_rate', '-')} °/s"),
                ("cant / flinch", f"{d.get('cant', '-')}° / {d.get('flinch', '-')}°")]
        for i, (k, v) in enumerate(rows):
            tx_ax.text(0.0, 0.92 - 0.16 * i, k, color=INK2, fontsize=9, transform=tx_ax.transAxes)
            tx_ax.text(0.42, 0.92 - 0.16 * i, v, color=INK, fontsize=9, transform=tx_ax.transAxes)
        tx_ax.text(0.0, -0.08, "trigger and IMU data from the weapon's SHOT_DETAIL, merged by the tracker",
                   color=MUTED, fontsize=7.5, transform=tx_ax.transAxes)
    else:
        at_ax.axis("off")
        at_ax.text(0.0, 0.5, "After-action panel: waiting for the first shot", color=INK2,
                   transform=at_ax.transAxes)
    st = [x for x in s.status if s.t(x) <= at]
    stat = (f"tracker {st[-1]['fps_x10']:.0f} fps, worst frame {st[-1]['worst_ms']:.1f} ms, {fault_text(st[-1]['faults'])}"
            if st else "tracker status: -")
    fig.suptitle(f"Aim-point tracker lane view (simulated session)    {stat}    "
                 f"hits {sum(1 for x in shown if x['result'] == 0)}/{len(shown)}", x=0.03, ha="left",
                 fontsize=11, fontweight="bold", color=INK)
    fig.savefig(path)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--png")
    ap.add_argument("--at", type=float)
    ap.add_argument("--frames")
    ap.add_argument("--fps", type=float, default=10.0)
    args = ap.parse_args(argv)
    style()
    s = Session(pathlib.Path(args.session))
    if args.png:
        draw(s, args.at if args.at is not None else s.end, pathlib.Path(args.png))
        print("->", args.png)
    if args.frames:
        d = pathlib.Path(args.frames)
        d.mkdir(parents=True, exist_ok=True)
        n = int(s.end * args.fps) + 1
        for i in range(n):
            draw(s, i / args.fps, d / f"frame_{i:04d}.png")
        print(f"{n} frames -> {d}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
