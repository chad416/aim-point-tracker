/*
 * Screen calibration from a projected dot grid (SPEC F05, §6.4).
 *
 *  1. Order: match the detected dots to the grid. The four extreme dots are
 *     the grid's corners (the camera must be roughly upright); a homography
 *     through them predicts every grid point, each takes the nearest dot
 *     inside a gate, and a homography through all matches repeats it once.
 *  2. Fit: Levenberg-Marquardt on the screen -> camera homography (8) and the
 *     distortion k1, k2, p1, p2 (4), minimising the reprojection error in
 *     pixels. Focal length and principal point stay at their nominal values;
 *     the homography absorbs their error. Every holdout_every-th grid point is
 *     kept out of the fit and used to check it.
 *  3. Accept only if the fit and hold-out residuals are inside P06.
 *
 * No heap: at most AT_CAL_MAX_POINTS grid points, worked on the stack
 * (under 8 kB at the deepest call).
 */
#ifndef AIMTRK_CALIB_H_
#define AIMTRK_CALIB_H_

#include "aimtrk/geom.h"

#define AT_CAL_MAX_POINTS 64

/* at_calib_report.status */
#define AT_CAL_OK           0
#define AT_CAL_TOO_FEW      (-1) /* fewer grid points found than min_found, or too few to fit */
#define AT_CAL_ORDER        (-2) /* corners not found or grid not matched */
#define AT_CAL_NO_CONVERGE  (-3) /* the fit did not converge */
#define AT_CAL_RESIDUAL     (-4) /* residuals over the P06 limits */

typedef struct {
	double max_fit_rms_px;     /* P06: 0.3 */
	double max_holdout_rms_px; /* P06: 0.5 */
	double gate_px;            /* ordering: a dot must lie this close to its predicted grid point */
	int min_found;             /* grid points that must be found; 0 = all */
	int holdout_every;         /* every n-th grid point is held out; 0 = none */
	int max_iter;              /* Levenberg-Marquardt iterations */
} at_calib_cfg;

typedef struct {
	int status; /* AT_CAL_* */
	int n_found, n_fit, n_holdout;
	int iterations;
	double fit_rms_px;     /* reprojection RMS of the fitted points */
	double holdout_rms_px; /* reprojection RMS of the held-out points (0 if none) */
	double max_px;         /* largest reprojection error, all found points */
} at_calib_report;

void at_calib_cfg_default(at_calib_cfg *cfg);

/*
 * Match detected dots img[2 * n_img] (pixels) to the grid grid[2 * cols * rows]
 * (screen mm, any order). match[i] receives the index of the dot for grid
 * point i, or -1. lens gives the nominal focal length and principal point.
 * Returns the number of grid points matched, or AT_CAL_ORDER.
 */
int at_calib_order(const double *img, int n_img, const double *grid, int n_grid, const at_lens *lens,
		   double gate_px, int *match);

/*
 * Fit the geometry to matched points (match from at_calib_order). On
 * AT_CAL_OK, *out is the new geometry; otherwise *out is left unchanged (the
 * previous calibration stays in use). The report is always filled in.
 */
int at_calib_fit(const double *img, const double *grid, const int *match, int n_grid,
		 const at_lens *nominal, const at_calib_cfg *cfg, double screen_w, double screen_h,
		 at_geom *out, at_calib_report *rep);

/* at_calib_order() then at_calib_fit(). */
int at_calib_run(const double *img, int n_img, const double *grid, int n_grid, const at_lens *nominal,
		 const at_calib_cfg *cfg, double screen_w, double screen_h, at_geom *out,
		 at_calib_report *rep);

#endif /* AIMTRK_CALIB_H_ */
