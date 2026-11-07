/*
 * The reference lane for the geometry tests (SPEC §2a, as sim/camsim.py):
 * a camera 0.75 m above the screen centre and 4 m out, looking at it, with
 * mild distortion; and the 9 x 6 calibration grid.
 */
#ifndef TEST_SCENE_H_
#define TEST_SCENE_H_

#include <math.h>

#include "aimtrk/geom.h"

#define TS_LENS {.f = 1600.0, .cx = 639.5, .cy = 399.5, .k1 = -0.08, .k2 = 0.01, .p1 = 0.0005, .p2 = -0.0003}
#define TS_SCREEN_W 3000.0
#define TS_SCREEN_H 1690.0
#define TS_COLS     9
#define TS_ROWS     6

/* Screen mm -> undistorted normalised camera coordinates of the reference camera. */
static inline void ts_camera_h(at_h33 *to_cam)
{
	double c[3] = {0.0, 750.0, 4000.0};
	double f[3] = {-c[0], -c[1], -c[2]};
	double nf = sqrt((f[0] * f[0]) + (f[1] * f[1]) + (f[2] * f[2]));

	for (int i = 0; i < 3; i++) {
		f[i] /= nf;
	}
	double r[3] = {-f[2], 0.0, f[0]}; /* f x (0, 1, 0) */
	double nr = sqrt((r[0] * r[0]) + (r[2] * r[2]));

	r[0] /= nr;
	r[2] /= nr;
	double d[3] = {(f[1] * r[2]) - (f[2] * r[1]), (f[2] * r[0]) - (f[0] * r[2]),
		       (f[0] * r[1]) - (f[1] * r[0])}; /* f x r */
	const double *rows[3] = {r, d, f};

	/* (X, Y, 0) -> R (P - C): columns R e_x, R e_y, -R C. */
	for (int i = 0; i < 3; i++) {
		to_cam->m[(i * 3) + 0] = rows[i][0];
		to_cam->m[(i * 3) + 1] = rows[i][1];
		to_cam->m[(i * 3) + 2] = -((rows[i][0] * c[0]) + (rows[i][1] * c[1]) + (rows[i][2] * c[2]));
	}
}

/* The calibration grid, row-major from the top-left, as camsim.calibration_grid(). */
static inline void ts_grid(double *grid)
{
	for (int r = 0; r < TS_ROWS; r++) {
		for (int c = 0; c < TS_COLS; c++) {
			grid[2 * ((r * TS_COLS) + c)] = -1350.0 + (2700.0 * c / (TS_COLS - 1));
			grid[(2 * ((r * TS_COLS) + c)) + 1] = 750.0 - (1500.0 * r / (TS_ROWS - 1));
		}
	}
}

#endif /* TEST_SCENE_H_ */
