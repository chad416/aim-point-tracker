/*
 * SPEC F05, §6.4, P06: grid ordering, the calibration fit, hold-out check,
 * and refusal of bad grids (the previous calibration stays in use).
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "aimtrk/calib.h"
#include "test_scene.h"
#include "test_util.h"

#define NG (TS_COLS * TS_ROWS)

ZTEST_SUITE(calib, NULL, NULL, NULL, NULL, NULL);

static double grid[2 * NG];
static double dots[2 * (NG + 8)];
static at_geom truth;
/* The calibration assumes the data-sheet focal length and the sensor centre (both a little off). */
static const at_lens nominal = {.f = 1590.0, .cx = 636.0, .cy = 402.0};

/* Image positions of the grid through the true geometry, with noise; returns the dot count. */
static int make_dots(double noise_px, uint32_t seed)
{
	at_lens l = TS_LENS;
	at_h33 h;

	ts_camera_h(&h);
	zassert_ok(at_geom_init(&truth, &l, &h, TS_SCREEN_W, TS_SCREEN_H));
	ts_grid(grid);
	for (int i = 0; i < NG; i++) {
		zassert_true(at_geom_to_image(&truth, grid[2 * i], grid[(2 * i) + 1], &dots[2 * i],
					      &dots[(2 * i) + 1]));
		dots[2 * i] += noise_px * tu_randn(&seed);
		dots[(2 * i) + 1] += noise_px * tu_randn(&seed);
	}
	return NG;
}

/* Largest screen error of a geometry against the truth, over the screen and its border. */
static double worst_mm(const at_geom *g)
{
	double worst = 0.0;

	for (int i = -10; i <= 10; i++) {
		for (int j = -6; j <= 6; j++) {
			double u, v, x, y;
			double px = 148.0 * i;
			double py = 139.0 * j;

			zassert_true(at_geom_to_image(&truth, px, py, &u, &v));
			zassert_not_equal(at_geom_to_screen(g, u, v, &x, &y), AT_GEOM_FAIL);
			worst = fmax(worst, hypot(x - px, y - py));
		}
	}
	return worst;
}

ZTEST(calib, test_fit_noise_free)
{
	at_calib_cfg cfg;
	at_calib_report rep;
	at_geom g;

	at_calib_cfg_default(&cfg);
	int n = make_dots(0.0, 1U);

	zassert_equal(at_calib_run(dots, n, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g, &rep),
		      AT_CAL_OK);
	TC_PRINT("noise-free: fit %.5f px, held-out %.5f px, %d iterations, worst %.4f mm\n",
		 rep.fit_rms_px, rep.holdout_rms_px, rep.iterations, worst_mm(&g));
	zassert_equal(rep.n_found, NG);
	zassert_equal(rep.n_holdout, 10, "every 5th of 54");
	zassert_equal(rep.n_fit, 44);
	/* The nominal focal length and principal point are off; the fit absorbs it. */
	zassert_true(rep.fit_rms_px < 0.01, "%f", rep.fit_rms_px);
	zassert_true(rep.holdout_rms_px < 0.01, "%f", rep.holdout_rms_px);
	zassert_true(worst_mm(&g) < 0.05);
}

/* 0.05 px dot noise: residuals at the noise level, the held-out points close to it. */
ZTEST(calib, test_fit_with_noise)
{
	at_calib_cfg cfg;
	at_calib_report rep;
	at_geom g;

	at_calib_cfg_default(&cfg);
	int n = make_dots(0.05, 2U);

	zassert_equal(at_calib_run(dots, n, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g, &rep),
		      AT_CAL_OK);
	TC_PRINT("0.05 px noise: fit %.4f px, held-out %.4f px, worst %.3f mm\n", rep.fit_rms_px,
		 rep.holdout_rms_px, worst_mm(&g));
	zassert_within(rep.fit_rms_px, 0.05 * sqrt(2.0) * sqrt((88.0 - 12.0) / 88.0), 0.02);
	zassert_true(rep.holdout_rms_px < 0.12);
	/* Beyond the grid (the screen corners) the fit extrapolates: the worst point is there. */
	zassert_true(worst_mm(&g) < 1.0);
}

/* Dots in any order, plus spurious detections away from the grid: same result. */
ZTEST(calib, test_order_shuffled_with_extras)
{
	at_calib_cfg cfg;
	at_calib_report rep, rep2;
	at_geom g, g2;
	uint32_t seed = 77U;
	int n = make_dots(0.0, 3U);

	at_calib_cfg_default(&cfg);
	zassert_equal(at_calib_run(dots, n, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g, &rep),
		      AT_CAL_OK);
	for (int i = n - 1; i > 0; i--) {
		int j = (int)(tu_rand(&seed) % (uint32_t)(i + 1));
		double tx = dots[2 * i], ty = dots[(2 * i) + 1];

		dots[2 * i] = dots[2 * j];
		dots[(2 * i) + 1] = dots[(2 * j) + 1];
		dots[2 * j] = tx;
		dots[(2 * j) + 1] = ty;
	}
	/* Halfway between grid columns: outside every gate, inside the grid's extent. */
	for (int k = 0; k < 4; k++) {
		double u, v;

		zassert_true(at_geom_to_image(&truth, -1181.25 + (675.0 * k), 450.0 - (200.0 * k), &u, &v));
		dots[2 * (n + k)] = u;
		dots[(2 * (n + k)) + 1] = v;
	}
	zassert_equal(at_calib_run(dots, n + 4, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g2,
				   &rep2),
		      AT_CAL_OK);
	zassert_within(rep2.fit_rms_px, rep.fit_rms_px, 1e-9);
	for (int i = 0; i < 9; i++) {
		zassert_within(g2.to_cam.m[i], g.to_cam.m[i], 1e-9 * fmax(1.0, fabs(g.to_cam.m[i])));
	}

	int match[NG];

	zassert_equal(at_calib_order(dots, n + 4, grid, NG, &nominal, cfg.gate_px, match), NG);
	for (int i = 0; i < NG; i++) {
		zassert_true(match[i] < n, "grid point %d took a spurious dot", i);
	}
}

/* Missing dots: refused unless min_found allows it; the old calibration is kept. */
ZTEST(calib, test_missing_dots)
{
	at_calib_cfg cfg;
	at_calib_report rep;
	at_geom g, before;
	int n = make_dots(0.0, 4U);

	at_calib_cfg_default(&cfg);
	memset(&before, 0xA5, sizeof(before));
	g = before;
	/* Drop grid point 20 (not a corner): move the last dot into its place. */
	dots[2 * 20] = dots[2 * (n - 1)];
	dots[(2 * 20) + 1] = dots[(2 * (n - 1)) + 1];
	zassert_equal(at_calib_run(dots, n - 1, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g,
				   &rep),
		      AT_CAL_TOO_FEW);
	zassert_equal(rep.n_found, NG - 1);
	zassert_mem_equal(&g, &before, sizeof(g), "previous calibration kept");

	cfg.min_found = NG - 2;
	zassert_equal(at_calib_run(dots, n - 1, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g,
				   &rep),
		      AT_CAL_OK);

	/* Too few dots to order at all. */
	zassert_equal(at_calib_run(dots, 3, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g, &rep),
		      AT_CAL_ORDER);
	/* The four corners and two more: ordered, but too few for the fit (12 parameters). */
	static const int keep[6] = {0, 8, 45, 53, 20, 30};

	n = make_dots(0.0, 4U);
	for (int k = 0; k < 6; k++) {
		dots[2 * k] = dots[2 * keep[k]];
		dots[(2 * k) + 1] = dots[(2 * keep[k]) + 1];
	}
	cfg.min_found = 4;
	zassert_equal(at_calib_run(dots, 6, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g, &rep),
		      AT_CAL_TOO_FEW);
	zassert_equal(rep.n_found, 6);
}

/* A grid that does not fit the model (dots displaced by 3 px RMS): residual over P06, refused. */
ZTEST(calib, test_bad_grid_refused)
{
	at_calib_cfg cfg;
	at_calib_report rep;
	at_geom g, before;
	int n = make_dots(3.0, 5U);

	at_calib_cfg_default(&cfg);
	memset(&before, 0x5A, sizeof(before));
	g = before;
	zassert_equal(at_calib_run(dots, n, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g, &rep),
		      AT_CAL_RESIDUAL);
	TC_PRINT("bad grid: fit %.3f px, held-out %.3f px\n", rep.fit_rms_px, rep.holdout_rms_px);
	zassert_true(rep.fit_rms_px > 0.3);
	zassert_mem_equal(&g, &before, sizeof(g));
}

ZTEST(calib, test_no_holdout_and_iteration_limit)
{
	at_calib_cfg cfg;
	at_calib_report rep;
	at_geom g;
	int n = make_dots(0.02, 6U);

	at_calib_cfg_default(&cfg);
	cfg.holdout_every = 0;
	zassert_equal(at_calib_run(dots, n, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g, &rep),
		      AT_CAL_OK);
	zassert_equal(rep.n_holdout, 0);
	zassert_equal(rep.n_fit, NG);
	zassert_equal(rep.holdout_rms_px, 0.0);

	cfg.max_iter = 1;
	zassert_equal(at_calib_run(dots, n, grid, NG, &nominal, &cfg, TS_SCREEN_W, TS_SCREEN_H, &g, &rep),
		      AT_CAL_NO_CONVERGE);
}
