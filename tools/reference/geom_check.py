"""C geometry and calibration against OpenCV (SPEC §9.2).

  distortion    at_lens_distort vs cv2.projectPoints (same Brown-Conrady model)
  undistortion  at_lens_undistort vs cv2.undistortPoints (iterated to 1e-15)
  homography    at_h33_dlt vs the true homography and cv2.findHomography,
                exact data, then noisy data
  calibration   at_calib_run vs cv2.calibrateCamera on the same grid points,
                noise-free and with 0.02 px noise; intrinsics fixed at the
                true values for both (so OpenCV's pose model and the C
                homography model both contain the truth)

Called by crosscheck.py; returns (results dict, list of errors).
"""
from __future__ import annotations

import ctypes as C

import cv2
import numpy as np

import aimtrk_c as AT

TOL_DISTORT = 1e-12  # normalised units
TOL_UNDISTORT = 1e-9
TOL_H_EXACT_PX = 1e-9


CRIT = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-15)


def _undistort(px, K, dist, P=None):
    """cv2.undistortPoints, iterated to 1e-15 (OpenCV 5 takes the criteria directly)."""
    pts = np.asarray(px, dtype=np.float64).reshape(-1, 1, 2)
    if P is None:
        return cv2.undistortPoints(pts, K, dist, criteria=CRIT)
    return cv2.undistortPoints(pts, K, dist, P=P, criteria=CRIT)


def _lens(lane) -> AT.Lens:
    return AT.lens_from(lane.f_px, lane.cx, lane.cy, (lane.k1, lane.k2, lane.p1, lane.p2))


def _cv_cam(lens: AT.Lens):
    k = np.array([[lens.f, 0, lens.cx], [0, lens.f, lens.cy], [0, 0, 1]], dtype=np.float64)
    d = np.array([lens.k1, lens.k2, lens.p1, lens.p2, 0.0])
    return k, d


def check(lib, lane, rng: np.random.Generator) -> tuple[dict, list[str]]:
    errs, res = [], {}
    lens = _lens(lane)
    K, dist = _cv_cam(lens)

    # Distortion and its inverse, over the frame (|x|, |y| <= 0.45 normalised).
    pts = rng.uniform(-0.45, 0.45, (1000, 2))
    cv_px = cv2.projectPoints(np.column_stack([pts, np.ones(len(pts))]), np.zeros(3), np.zeros(3), K, dist)[0]
    cv_d = (cv_px.reshape(-1, 2) - [lens.cx, lens.cy]) / lens.f
    xd, yd = C.c_double(), C.c_double()
    worst_d = worst_u = 0.0
    und = _undistort(cv_px, K, dist).reshape(-1, 2)
    for i, (x, y) in enumerate(pts):
        lib.at_lens_distort(C.byref(lens), x, y, C.byref(xd), C.byref(yd))
        worst_d = max(worst_d, abs(xd.value - cv_d[i, 0]), abs(yd.value - cv_d[i, 1]))
        xn, yn = C.c_double(), C.c_double()
        if not lib.at_lens_undistort(C.byref(lens), cv_d[i, 0], cv_d[i, 1], C.byref(xn), C.byref(yn)):
            errs.append(f"undistort failed at {cv_d[i]}")
            continue
        worst_u = max(worst_u, abs(xn.value - und[i, 0]), abs(yn.value - und[i, 1]))
    res["distort_max_diff"] = float(worst_d)
    res["undistort_max_diff_vs_opencv"] = float(worst_u)
    if worst_d > TOL_DISTORT:
        errs.append(f"distortion differs from OpenCV by {worst_d:.3g}")
    if worst_u > TOL_UNDISTORT:
        errs.append(f"undistortion differs from OpenCV by {worst_u:.3g}")

    # Homography: screen mm -> pixel-like coordinates of a keystoned view.
    h_true = np.array([[0.39, 0.01, 640.0], [0.002, -0.37, 400.0], [1e-6, -4e-5, 1.0]])
    src = rng.uniform([-1500, -845], [1500, 845], (40, 2))
    dst = cv2.perspectiveTransform(src.reshape(-1, 1, 2), h_true).reshape(-1, 2)
    test = rng.uniform([-1500, -845], [1500, 845], (200, 2)).reshape(-1, 1, 2)
    ref = cv2.perspectiveTransform(test, h_true)
    for label, d in (("exact", dst), ("noisy", dst + rng.normal(0, 0.1, dst.shape))):
        h = AT.H33()
        if lib.at_h33_dlt(AT._darr(src), AT._darr(d), len(src), C.byref(h)) != 0:
            errs.append(f"DLT failed ({label})")
            continue
        out_c = cv2.perspectiveTransform(test, np.array(h.m).reshape(3, 3))
        out_cv = cv2.perspectiveTransform(test, cv2.findHomography(src, d, 0)[0])
        res[f"homography_{label}"] = {"c_vs_true_px": float(np.abs(out_c - ref).max()),
                                      "opencv_vs_true_px": float(np.abs(out_cv - ref).max()),
                                      "c_vs_opencv_px": float(np.abs(out_c - out_cv).max())}
    # Exact data: the C DLT must reproduce the truth; OpenCV's least-squares
    # refinement stops near 1e-5 px, so C vs OpenCV is held to 1e-4 px.
    ex = res.get("homography_exact", {})
    if ex.get("c_vs_true_px", 1.0) > TOL_H_EXACT_PX:
        errs.append(f"homography (exact data) off the truth by {ex.get('c_vs_true_px')} px")
    if ex.get("c_vs_opencv_px", 1.0) > 1e-4:
        errs.append(f"homography (exact data) differs from OpenCV by {ex.get('c_vs_opencv_px')} px")
    # Noisy data: C is the algebraic DLT, OpenCV minimises the geometric error,
    # so they differ at a fraction of the 0.1 px noise; only more is an error.
    if res.get("homography_noisy", {}).get("c_vs_opencv_px", 1.0) > 0.05:
        errs.append(f"homography (noisy data) differs from OpenCV: {res.get('homography_noisy')}")

    # Calibration on the grid, both with the true intrinsics: noise-free, then 0.02 px dot noise.
    for noise, tol_px, tol_mm in ((0.0, 1e-3, 0.005), (0.02, 0.02, 0.1)):
        r = _calib_pair(lib, lane, lens, noise, rng)
        res[f"calibration_noise_{noise}px"] = r
        if "error" in r:
            errs.append(r["error"])
            continue
        if r["reprojection_max_diff_px"] > tol_px:
            errs.append(f"calibration ({noise} px noise): reprojection differs from OpenCV by "
                        f"{r['reprojection_max_diff_px']} px")
        if r["screen_max_diff_mm"] > tol_mm:
            errs.append(f"calibration ({noise} px noise): screen mapping differs from OpenCV by "
                        f"{r['screen_max_diff_mm']} mm")
    res["k2_over_20_noise_draws"] = _k2_spread(lib, lane, lens, rng)
    return res, errs


def _k2_spread(lib, lane, lens: AT.Lens, rng, draws: int = 20, noise: float = 0.02) -> dict:
    """How well one plane determines k2: C fits on 20 draws of 0.02 px dot noise (true intrinsics)."""
    import camsim
    grid = camsim.calibration_grid(lane)
    nominal = AT.lens_from(lens.f, lens.cx, lens.cy)
    k2, worst_mm = [], 0.0
    test = rng.uniform(grid.min(axis=0), grid.max(axis=0), (200, 2))
    for _ in range(draws):
        img = lane.screen_to_image(grid) + rng.normal(0, noise, (len(grid), 2))
        g, _ = AT.calibrate(lib, img, grid, nominal, lane.screen_w, lane.screen_h)
        if g is None:
            continue
        k2.append(g.lens.k2)
        for x, y in test:
            u, v = lane.screen_to_image((x, y))[0]
            sx, sy, _ = AT.to_screen(lib, g, u, v)
            worst_mm = max(worst_mm, float(np.hypot(sx - x, sy - y)))
    return {"true_k2": lane.k2, "fits": len(k2), "k2_min": round(min(k2), 5), "k2_max": round(max(k2), 5),
            "k2_std": round(float(np.std(k2)), 5), "worst_mapping_error_mm_inside_grid": round(worst_mm, 4)}


def _calib_pair(lib, lane, lens: AT.Lens, noise: float, rng) -> dict:
    """at_calib_run and cv2.calibrateCamera on the same grid points; how far apart they are."""
    import camsim
    grid = camsim.calibration_grid(lane)
    img = lane.screen_to_image(grid) + rng.normal(0, noise, (len(grid), 2))
    nominal = AT.lens_from(lens.f, lens.cx, lens.cy)
    cfg = AT.CalibCfg()
    lib.at_calib_cfg_default(C.byref(cfg))
    cfg.holdout_every = 0
    g, rep = AT.calibrate(lib, img, grid, nominal, lane.screen_w, lane.screen_h, cfg)
    if g is None:
        return {"error": f"C calibration refused ({noise} px noise): {rep}"}
    obj = np.column_stack([grid, np.zeros(len(grid))])
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_FOCAL_LENGTH | cv2.CALIB_FIX_PRINCIPAL_POINT
             | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_K3)
    K0 = np.array([[lens.f, 0, lens.cx], [0, lens.f, lens.cy], [0, 0, 1]])
    _, Kc, dc, rv, tv = cv2.calibrateCamera([obj.astype(np.float32)], [img.astype(np.float32)],
                                            (lane.width, lane.height), K0.copy(), np.zeros(5), flags=flags,
                                            criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-15))
    cv_proj = cv2.projectPoints(obj, rv[0], tv[0], Kc, dc)[0].reshape(-1, 2)
    c_proj = np.array([AT.to_image(lib, g, x, y) for x, y in grid])
    # Screen mapping over the grid's extent: C pixel -> screen vs OpenCV's pose.
    R, _ = cv2.Rodrigues(rv[0])
    h_cv = Kc @ np.column_stack([R[:, 0], R[:, 1], tv[0].ravel()])  # screen -> undistorted pixel
    worst_mm = 0.0
    for x, y in rng.uniform(grid.min(axis=0), grid.max(axis=0), (300, 2)):
        u, v = AT.to_image(lib, g, x, y)
        s = cv2.perspectiveTransform(_undistort([[u, v]], Kc, dc, Kc), np.linalg.inv(h_cv)).ravel()
        worst_mm = max(worst_mm, float(np.hypot(s[0] - x, s[1] - y)))
    return {
        "c_rms_px": round(float(np.sqrt(np.mean(np.sum((c_proj - img) ** 2, axis=1)))), 5),
        "opencv_rms_px": round(float(np.sqrt(np.mean(np.sum((cv_proj - img) ** 2, axis=1)))), 5),
        "reprojection_max_diff_px": round(float(np.abs(c_proj - cv_proj).max()), 5),
        "screen_max_diff_mm": round(worst_mm, 5),
        "c_k": [round(getattr(g.lens, n), 6) for n in ("k1", "k2", "p1", "p2")],
        "opencv_k": [round(float(v), 6) for v in dc.ravel()[:4]],
        "true_k": [lane.k1, lane.k2, lane.p1, lane.p2],
    }
