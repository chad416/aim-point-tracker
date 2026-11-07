#!/usr/bin/env python3
"""Camera simulator (SPEC §8): the projection screen as the lane's IR camera sees it.

Week 1 scope: geometry (pinhole camera + Brown-Conrady distortion looking at
the screen plane), emitter spots (Gaussian PSF, energy from the pulse's
overlap with the exposure, smear along the aim motion), the sensor (shot,
read and fixed-pattern noise, hot pixels, 8-bit quantisation, saturation) and
ground truth for every spot. Projector leak, reflections, a sunlit window and
aim motion from S1's IMU recordings come with the interference set (week 3).

Coordinates
  screen  x right, y up, mm, origin at the screen centre (SPEC §6.3)
  world   screen plane z = 0, z towards the shooters, mm
  image   x right, y down, px, pixel centres at integers (aimtrk/centroid.h)

Frame stream: each frame is a 32-byte header (FRAME_HEADER) followed by
width * height bytes, row-major. Ground truth goes to a JSON lines file, one
line per spot.

Usage:
  camsim.py lane                       reference lane check (SPEC §2a) -> results/lane_check.json
  camsim.py bench [--frames 20]        render time per frame and pipe throughput -> results/camsim_bench.json
  camsim.py render --frames N --out frames.bin --truth truth.jsonl [--spots 3] [--seed 1]
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import pathlib
import struct
import subprocess
import sys
import time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]

# magic, width, height, frame index, exposure start and end (lane time, us), flags
FRAME_HEADER = struct.Struct("<4sHHIqqI")
FRAME_MAGIC = b"AIMF"

FRAME_PERIOD_US = 10_000   # 100 fps, one frame per SYNC period (SPEC §2a)
EXPOSURE_OFFSET_US = 4_500  # exposure k starts at k * 10 ms + 4.5 ms
EXPOSURE_US = 1_000


def exposure_window(k: int) -> tuple[int, int]:
    """Exposure of frame k in lane time, us."""
    t0 = k * FRAME_PERIOD_US + EXPOSURE_OFFSET_US
    return t0, t0 + EXPOSURE_US


@dataclasses.dataclass
class Lane:
    """Reference lane, SPEC §2a. Lengths in mm. The lens values are assumptions."""
    screen_w: float = 3000.0
    screen_h: float = 1690.0
    cam_pos: tuple[float, float, float] = (0.0, 750.0, 4000.0)  # above the shooters, 4 m out
    look_at: tuple[float, float, float] = (0.0, 0.0, 0.0)       # screen centre
    width: int = 1280
    height: int = 800
    f_px: float = 1600.0  # about 3.2 m across at 4 m
    k1: float = -0.08     # mild barrel distortion
    k2: float = 0.01
    p1: float = 0.0005
    p2: float = -0.0003

    @property
    def cx(self) -> float:
        return (self.width - 1) / 2.0

    @property
    def cy(self) -> float:
        return (self.height - 1) / 2.0

    def rotation(self) -> np.ndarray:
        """World -> camera rotation; camera axes x right, y down, z forward."""
        c = np.asarray(self.cam_pos)
        f = np.asarray(self.look_at) - c
        f /= np.linalg.norm(f)
        r = np.cross(f, [0.0, 1.0, 0.0])
        r /= np.linalg.norm(r)
        d = np.cross(f, r)
        return np.vstack([r, d, f])

    def distort(self, xn: np.ndarray, yn: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        r2 = xn * xn + yn * yn
        radial = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
        xd = xn * radial + 2.0 * self.p1 * xn * yn + self.p2 * (r2 + 2.0 * xn * xn)
        yd = yn * radial + self.p1 * (r2 + 2.0 * yn * yn) + 2.0 * self.p2 * xn * yn
        return xd, yd

    def screen_to_image(self, xy_mm: np.ndarray) -> np.ndarray:
        """Screen points (N, 2) mm -> image points (N, 2) px."""
        xy = np.atleast_2d(np.asarray(xy_mm, dtype=float))
        world = np.column_stack([xy, np.zeros(len(xy))])
        cam = (world - np.asarray(self.cam_pos)) @ self.rotation().T
        xd, yd = self.distort(cam[:, 0] / cam[:, 2], cam[:, 1] / cam[:, 2])
        return np.column_stack([self.f_px * xd + self.cx, self.f_px * yd + self.cy])

    def jacobian(self, xy_mm: np.ndarray, h: float = 1.0) -> np.ndarray:
        """d(image px) / d(screen mm) at one screen point, 2 x 2, by central differences."""
        p = np.asarray(xy_mm, dtype=float)
        cols = []
        for k in range(2):
            e = np.zeros(2)
            e[k] = h
            cols.append((self.screen_to_image(p + e)[0] - self.screen_to_image(p - e)[0]) / (2 * h))
        return np.column_stack(cols)


@dataclasses.dataclass
class Sensor:
    """1 MP global-shutter monochrome sensor, 8-bit output. Values are assumptions for a
    sensor of this class, not a datasheet."""
    full_well_e: float = 10_000.0  # electrons at 255 DN above black
    read_noise_e: float = 12.0
    black_dn: float = 4.0
    prnu: float = 0.01              # pixel response non-uniformity, relative (fixed pattern)
    dsnu_dn: float = 0.3            # dark signal non-uniformity, DN (fixed pattern)
    hot_fraction: float = 2e-5      # hot pixels, fraction of all pixels
    hot_dn: tuple[float, float] = (40.0, 200.0)
    ambient_dn: float = 12.0        # room IR through the 850 nm filter in 1 ms, DN
    ambient_gradient: float = 0.3   # relative left-to-right change of the ambient level
    seed: int = 1234                # fixed-pattern noise and hot pixels belong to the sensor

    @property
    def e_per_dn(self) -> float:
        return self.full_well_e / 255.0

    def background_noise_dn(self) -> float:
        """Temporal + fixed-pattern noise of the flat ambient level, DN (before quantisation)."""
        sig_e = self.ambient_dn * self.e_per_dn
        shot = math.sqrt(sig_e) / self.e_per_dn
        read = self.read_noise_e / self.e_per_dn
        prnu = self.prnu * self.ambient_dn
        return math.sqrt(shot**2 + read**2 + prnu**2 + self.dsnu_dn**2)


@dataclasses.dataclass
class Spot:
    """One emitter pulse as seen by the camera."""
    xy_mm: tuple[float, float]           # screen position at the start of the pulse
    total_dn: float                      # integrated signal of the full pulse, DN * px
    t_us: tuple[int, int] | None = None  # pulse start, end (lane time); None = fully inside the exposure
    vel_mm_s: tuple[float, float] = (0.0, 0.0)  # aim motion during the pulse
    sigma_mm: float = 2.8                # emitter spot size on the screen (about 10 mm across)
    weapon: int = 0
    kind: str = "test"                   # "aim", "shot" or "test"


class Camera:
    def __init__(self, lane: Lane | None = None, sensor: Sensor | None = None,
                 lens_sigma_px: float = 0.5, fast_noise: bool = False):
        self.lane = lane or Lane()
        self.sensor = sensor or Sensor()
        self.lens_sigma_px = lens_sigma_px
        self.fast_noise = fast_noise  # Gaussian approximation of shot noise (faster)
        h, w = self.lane.height, self.lane.width
        fp = np.random.default_rng(self.sensor.seed)
        self.prnu_map = (1.0 + self.sensor.prnu * fp.standard_normal((h, w))).astype(np.float32)
        self.dsnu_map = (self.sensor.dsnu_dn * fp.standard_normal((h, w))).astype(np.float32)
        n_hot = int(round(self.sensor.hot_fraction * h * w))
        self.hot_idx = fp.choice(h * w, size=n_hot, replace=False)
        self.hot_val = fp.uniform(*self.sensor.hot_dn, size=n_hot).astype(np.float32)
        x = np.linspace(-0.5, 0.5, w, dtype=np.float32)
        self.ambient = np.broadcast_to(
            self.sensor.ambient_dn * (1.0 + self.sensor.ambient_gradient * x), (h, w))

    def spot_cov_px(self, xy_mm, sigma_mm: float) -> np.ndarray:
        """Image covariance of a screen spot: the emitter spot through the local mapping, plus lens blur."""
        j = self.lane.jacobian(xy_mm)
        return (sigma_mm**2) * (j @ j.T) + (self.lens_sigma_px**2) * np.eye(2)

    def add_spot(self, img: np.ndarray, spot: Spot, exposure: tuple[int, int]) -> dict:
        """Add a spot's signal (DN, noise-free) to img. Returns its ground truth record."""
        t0, t1 = exposure
        ps, pe = spot.t_us if spot.t_us is not None else (t0, t1)
        o0, o1 = max(ps, t0), min(pe, t1)
        frac = max(o1 - o0, 0) / max(pe - ps, 1)
        vel = np.asarray(spot.vel_mm_s, dtype=float)
        start = np.asarray(spot.xy_mm, dtype=float)
        # Screen path during the part of the pulse inside the exposure.
        a = start + vel * (o0 - ps) * 1e-6
        b = start + vel * (o1 - ps) * 1e-6
        mean_mm = 0.5 * (a + b)
        rec = {"weapon": spot.weapon, "kind": spot.kind, "x_mm": float(mean_mm[0]),
               "y_mm": float(mean_mm[1]), "energy": float(frac), "total_dn": spot.total_dn * frac}
        uv_a, uv_b = self.lane.screen_to_image(np.vstack([a, b]))
        uv = self.lane.screen_to_image(mean_mm)[0]
        cov = self.spot_cov_px(mean_mm, spot.sigma_mm)
        rec.update(u=float(uv[0]), v=float(uv[1]),
                   peak_dn=spot.total_dn * frac / (2 * math.pi * math.sqrt(np.linalg.det(cov))))
        if frac <= 0.0:
            return rec
        n_sub = max(1, int(math.ceil(np.linalg.norm(uv_b - uv_a) * 4)))
        centres = [uv_a + (uv_b - uv_a) * ((i + 0.5) / n_sub) for i in range(n_sub)] if n_sub > 1 else [uv]
        render_gaussian(img, centres, cov, spot.total_dn * frac)
        return rec

    def render(self, spots: list[Spot], k: int, rng: np.random.Generator) -> tuple[np.ndarray, list[dict]]:
        """Frame k (uint8, height x width) with its ground truth."""
        exposure = exposure_window(k)
        signal = np.array(self.ambient, dtype=np.float32)
        truth = []
        for s in spots:
            rec = self.add_spot(signal, s, exposure)
            rec["frame"] = k
            truth.append(rec)
        img = self.expose(signal, rng)
        return img, truth

    def expose(self, signal_dn: np.ndarray, rng: np.random.Generator,
               rows: slice = slice(None), cols: slice = slice(None)) -> np.ndarray:
        """Sensor model on a noise-free signal (DN above black). rows/cols select the
        part of the sensor the signal covers, so patches get the right fixed pattern."""
        s = self.sensor
        e = signal_dn * self.prnu_map[rows, cols] * s.e_per_dn
        e = np.maximum(e, 0.0)
        if self.fast_noise:
            e = e + np.sqrt(e) * rng.standard_normal(e.shape, dtype=np.float32)
        else:
            e = rng.poisson(e).astype(np.float32)
        e += s.read_noise_e * rng.standard_normal(e.shape, dtype=np.float32)
        dn = e / s.e_per_dn + s.black_dn + self.dsnu_map[rows, cols]
        if rows == slice(None) and cols == slice(None):
            dn.reshape(-1)[self.hot_idx] += self.hot_val
        return np.clip(np.floor(dn + 0.5), 0, 255).astype(np.uint8)


def render_gaussian(img: np.ndarray, centres, cov: np.ndarray, total: float, ss: int = 5) -> None:
    """Add a 2-D Gaussian of integrated value `total` to img, integrated over each pixel by
    ss x ss supersampling and averaged over the given centres (smear)."""
    inv = np.linalg.inv(cov)
    norm = total / (2 * math.pi * math.sqrt(np.linalg.det(cov))) / len(centres)
    rad = int(math.ceil(5.0 * math.sqrt(max(np.linalg.eigvalsh(cov))))) + 1
    h, w = img.shape
    off = (np.arange(ss) + 0.5) / ss - 0.5
    for c in centres:
        x0, y0 = int(round(c[0])) - rad, int(round(c[1])) - rad
        xs = np.arange(x0, x0 + 2 * rad + 1)
        ys = np.arange(y0, y0 + 2 * rad + 1)
        keep_x = (xs >= 0) & (xs < w)
        keep_y = (ys >= 0) & (ys < h)
        if not keep_x.any() or not keep_y.any():
            continue
        # sub-sample positions relative to the centre
        dx = (xs[:, None] + off[None, :]).reshape(-1) - c[0]
        dy = (ys[:, None] + off[None, :]).reshape(-1) - c[1]
        DX, DY = np.meshgrid(dx, dy)
        q = inv[0, 0] * DX * DX + 2 * inv[0, 1] * DX * DY + inv[1, 1] * DY * DY
        g = np.exp(-0.5 * q).reshape(len(ys), ss, len(xs), ss).mean(axis=(1, 3)) * norm
        g = g[np.ix_(keep_y, keep_x)]
        img[ys[keep_y][0]:ys[keep_y][-1] + 1, xs[keep_x][0]:xs[keep_x][-1] + 1] += g.astype(img.dtype)


def write_frame(f, img: np.ndarray, k: int, flags: int = 0) -> None:
    t0, t1 = exposure_window(k)
    f.write(FRAME_HEADER.pack(FRAME_MAGIC, img.shape[1], img.shape[0], k, t0, t1, flags))
    f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())


def read_frames(path: pathlib.Path):
    """Yield (header dict, image) from a frame stream file."""
    with open(path, "rb") as f:
        while hdr := f.read(FRAME_HEADER.size):
            magic, w, h, k, t0, t1, flags = FRAME_HEADER.unpack(hdr)
            if magic != FRAME_MAGIC:
                raise ValueError(f"bad frame magic {magic!r}")
            img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
            yield {"frame": k, "t0_us": t0, "t1_us": t1, "flags": flags}, img


def random_spots(rng: np.random.Generator, lane: Lane, n: int, total_dn: float,
                 margin_mm: float = 30.0) -> list[Spot]:
    hw, hh = lane.screen_w / 2 - margin_mm, lane.screen_h / 2 - margin_mm
    return [Spot(xy_mm=(rng.uniform(-hw, hw), rng.uniform(-hh, hh)), total_dn=total_dn)
            for _ in range(n)]


# ---------------------------------------------------------------- calibration and zeroing scenes

# What the calibration assumes about the lens: data-sheet focal length and the
# sensor centre, no distortion. Deliberately not the simulator's true lens
# (Lane: f 1600 px, principal point at the centre, k1 -0.08 ...): the fit has
# to absorb the difference (SPEC §6.4).
NOMINAL_LENS = {"f": 1590.0, "cx": 636.0, "cy": 402.0}

GRID_COLS, GRID_ROWS = 9, 6
GRID_DOT_SIGMA_MM = 5.0   # projected calibration dots are larger than the emitter spot
GRID_DOT_TOTAL_DN = 2000.0  # long exposure: peak about 75 DN


def calibration_grid(lane: Lane, cols: int = GRID_COLS, rows: int = GRID_ROWS,
                     margin_x: float = 150.0, margin_y: float = 95.0) -> np.ndarray:
    """Screen positions (cols * rows, 2) mm of the projected dot grid, row-major from the top-left."""
    xs = np.linspace(-lane.screen_w / 2 + margin_x, lane.screen_w / 2 - margin_x, cols)
    ys = np.linspace(lane.screen_h / 2 - margin_y, -lane.screen_h / 2 + margin_y, rows)
    return np.array([(x, y) for y in ys for x in xs])


def grid_spots(grid_mm: np.ndarray, total_dn: float = GRID_DOT_TOTAL_DN,
               sigma_mm: float = GRID_DOT_SIGMA_MM) -> list[Spot]:
    return [Spot(xy_mm=(float(x), float(y)), total_dn=total_dn, sigma_mm=sigma_mm, kind="grid")
            for x, y in grid_mm]


def emitter_point(bore_xy_mm, mis_mrad=(0.0, 0.0), muzzle_mm=(0.0, 0.0, 5000.0)) -> np.ndarray:
    """Where the emitter's beam meets the screen when the bore points at bore_xy_mm.

    The emitter is turned from the bore by mis_mrad (right, up) about the
    muzzle; the beam is intersected with the screen plane z = 0. So the
    offset on the screen grows away from the screen centre, as on a real
    lane (SPEC F13).
    """
    m = np.asarray(muzzle_mm, dtype=float)
    b = np.array([bore_xy_mm[0], bore_xy_mm[1], 0.0]) - m
    b /= np.linalg.norm(b)
    right = np.cross(b, [0.0, 1.0, 0.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, b)
    az, el = mis_mrad[0] * 1e-3, mis_mrad[1] * 1e-3
    d = b + math.tan(az) * right + math.tan(el) * up
    t = -m[2] / d[2]
    return (m + t * d)[:2]


# ---------------------------------------------------------------- commands

def cmd_lane(args) -> int:
    """SPEC §2a check: does the screen fit, how many mm per pixel, how big is the spot."""
    lane, cam = Lane(), Camera()
    hw, hh = lane.screen_w / 2, lane.screen_h / 2
    pts = {"centre": (0.0, 0.0), "top_left": (-hw, hh), "top_right": (hw, hh),
           "bottom_left": (-hw, -hh), "bottom_right": (hw, -hh)}
    out = {"lane": dataclasses.asdict(lane), "sensor": dataclasses.asdict(Sensor()),
           "lens_sigma_px": cam.lens_sigma_px, "points": {}}
    for name, p in pts.items():
        uv = lane.screen_to_image(p)[0]
        j = lane.jacobian(p)
        sv = np.linalg.svd(j, compute_uv=False)       # px per mm along the principal directions
        cov = cam.spot_cov_px(p, Spot((0, 0), 0).sigma_mm)
        ev = np.sqrt(np.linalg.eigvalsh(cov))
        out["points"][name] = {
            "u": round(float(uv[0]), 1), "v": round(float(uv[1]), 1),
            "mm_per_px": [round(float(1 / sv[0]), 3), round(float(1 / sv[1]), 3)],
            "spot_sigma_px": [round(float(ev[0]), 3), round(float(ev[1]), 3)],
        }
    corners = np.array([[v["u"], v["v"]] for k, v in out["points"].items() if k != "centre"])
    out["margin_px"] = round(float(min(corners[:, 0].min(), corners[:, 1].min(),
                                       lane.width - 1 - corners[:, 0].max(),
                                       lane.height - 1 - corners[:, 1].max())), 1)
    out["background_noise_dn"] = round(Sensor().background_noise_dn(), 3)
    dst = ROOT / "results" / "lane_check.json"
    dst.write_text(json.dumps(out, indent=2) + "\n")
    for name, p in out["points"].items():
        print(f"{name:13s} u={p['u']:7.1f} v={p['v']:6.1f}  mm/px {p['mm_per_px']}  spot sigma px {p['spot_sigma_px']}")
    print(f"screen-to-frame margin {out['margin_px']} px; background noise {out['background_noise_dn']} DN -> {dst}")
    return 0 if out["margin_px"] > 0 else 1


def cmd_bench(args) -> int:
    """SPEC §12: render time per frame and pipe throughput."""
    res = {"frames": args.frames}
    rng = np.random.default_rng(1)
    for fast in (False, True):
        cam = Camera(fast_noise=fast)
        spots = random_spots(rng, cam.lane, 4, 3000.0)
        cam.render(spots, 0, rng)  # warm-up
        t = time.perf_counter()
        for k in range(args.frames):
            img, _ = cam.render(spots, k, rng)
        res["render_ms_" + ("gauss" if fast else "poisson")] = round(
            1e3 * (time.perf_counter() - t) / args.frames, 2)
    # Pipe: the same frame written args.frames * 10 times into `cat > /dev/null`.
    n = args.frames * 10
    p = subprocess.Popen(["cat"], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
    t = time.perf_counter()
    for k in range(n):
        write_frame(p.stdin, img, k)
    p.stdin.close()
    p.wait()
    dt = time.perf_counter() - t
    res["pipe_MB_s"] = round(n * (img.size + FRAME_HEADER.size) / dt / 1e6, 1)
    res["pipe_fps"] = round(n / dt, 1)
    dst = ROOT / "results" / "camsim_bench.json"
    dst.write_text(json.dumps(res, indent=2) + "\n")
    print(json.dumps(res, indent=2))
    return 0


def cmd_render(args) -> int:
    rng = np.random.default_rng(args.seed)
    cam = Camera(fast_noise=args.fast)
    spots = random_spots(rng, cam.lane, args.spots, args.total_dn)
    with open(args.out, "wb") as f, open(args.truth, "w") as t:
        for k in range(args.frames):
            img, truth = cam.render(spots if k >= args.empty else [], k, rng)
            write_frame(f, img, k)
            for rec in truth:
                t.write(json.dumps(rec) + "\n")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("lane")
    b = sub.add_parser("bench")
    b.add_argument("--frames", type=int, default=20)
    r = sub.add_parser("render")
    r.add_argument("--frames", type=int, default=10)
    r.add_argument("--empty", type=int, default=1, help="leading frames without spots (background)")
    r.add_argument("--spots", type=int, default=3)
    r.add_argument("--total-dn", type=float, default=1400.0, help="integrated spot signal, DN * px")
    r.add_argument("--seed", type=int, default=1)
    r.add_argument("--fast", action="store_true", help="Gaussian approximation of shot noise")
    r.add_argument("--out", required=True)
    r.add_argument("--truth", required=True)
    args = ap.parse_args(argv)
    return {"lane": cmd_lane, "bench": cmd_bench, "render": cmd_render}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
