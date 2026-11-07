/*
 * SPEC F04, §6.3: linear algebra helpers, lens distortion and its inverse,
 * homographies (normalised DLT), image <-> screen mapping, off-screen points.
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "aimtrk/geom.h"
#include "aimtrk/linalg.h"
#include "test_scene.h"
#include "test_util.h"

ZTEST_SUITE(geom, NULL, NULL, NULL, NULL, NULL);

static const at_lens lens_ref = TS_LENS;

ZTEST(geom, test_solve)
{
	/* A random well-conditioned 12 x 12 system with a known solution. */
	uint32_t seed = 3U;
	double a[144], a0[144], x[12], b[12];

	for (int i = 0; i < 144; i++) {
		a[i] = tu_unif(&seed) - 0.5;
	}
	for (int i = 0; i < 12; i++) {
		a[(i * 12) + i] += 4.0;
		x[i] = tu_unif(&seed) * 10.0;
	}
	memcpy(a0, a, sizeof(a));
	for (int r = 0; r < 12; r++) {
		b[r] = 0.0;
		for (int c = 0; c < 12; c++) {
			b[r] += a0[(r * 12) + c] * x[c];
		}
	}
	zassert_true(at_solve(a, b, 12));
	for (int i = 0; i < 12; i++) {
		zassert_within(b[i], x[i], 1e-10);
	}
	/* Singular: two equal rows. */
	double s[9] = {1, 2, 3, 1, 2, 3, 0, 1, 1};
	double sb[3] = {1, 1, 1};

	zassert_false(at_solve(s, sb, 3));
	double z[4] = {0};
	double zb[2] = {1, 1};

	zassert_false(at_solve(z, zb, 2));
}

ZTEST(geom, test_sym_eig)
{
	uint32_t seed = 11U;
	double a[81], a0[81], ev[9], v[81];

	for (int r = 0; r < 9; r++) {
		for (int c = r; c < 9; c++) {
			a[(r * 9) + c] = tu_unif(&seed) - 0.5;
			a[(c * 9) + r] = a[(r * 9) + c];
		}
	}
	memcpy(a0, a, sizeof(a));
	zassert_true(at_sym_eig(a, 9, ev, v));
	for (int k = 0; k < 9; k++) {
		if (k > 0) {
			zassert_true(ev[k] >= ev[k - 1], "ascending");
		}
		for (int r = 0; r < 9; r++) {
			double av = 0.0;

			for (int c = 0; c < 9; c++) {
				av += a0[(r * 9) + c] * v[(c * 9) + k];
			}
			zassert_within(av, ev[k] * v[(r * 9) + k], 1e-10, "A v = lambda v");
		}
		for (int j = 0; j < 9; j++) {
			double d = 0.0;

			for (int r = 0; r < 9; r++) {
				d += v[(r * 9) + k] * v[(r * 9) + j];
			}
			zassert_within(d, (j == k) ? 1.0 : 0.0, 1e-10, "orthonormal");
		}
	}
}

/* Undistortion inverts distortion over the whole field (|x|, |y| <= 0.5, the corners of the frame). */
ZTEST(geom, test_distortion_round_trip)
{
	double worst = 0.0;

	for (int i = 0; i <= 20; i++) {
		for (int j = 0; j <= 20; j++) {
			double xn = -0.5 + (0.05 * i);
			double yn = -0.5 + (0.05 * j);
			double xd, yd, x2, y2;

			at_lens_distort(&lens_ref, xn, yn, &xd, &yd);
			zassert_true(at_lens_undistort(&lens_ref, xd, yd, &x2, &y2));
			worst = fmax(worst, hypot(x2 - xn, y2 - yn));
		}
	}
	zassert_true(worst < 1e-11, "%g", worst); /* Newton stops at 1e-12 in distorted coordinates */

	/* No distortion: identity. */
	at_lens plain = {.f = 1000.0};
	double xd, yd;

	at_lens_distort(&plain, 0.3, -0.2, &xd, &yd);
	zassert_within(xd, 0.3, 1e-15);
	zassert_within(yd, -0.2, 1e-15);

	/* Where the model folds over (strong barrel, far out) there is no inverse. */
	at_lens fold = {.f = 1000.0, .k1 = -1.0};

	zassert_false(at_lens_undistort(&fold, 2.0, 0.0, &xd, &yd));
}

static at_h33 h_ref(void)
{
	return (at_h33){{1.2, 0.1, 30.0, -0.05, 0.9, -12.0, 1e-4, -2e-4, 1.0}};
}

ZTEST(geom, test_homography_apply_invert)
{
	at_h33 h = h_ref(), inv;
	double x, y, x2, y2;

	zassert_true(at_h33_invert(&h, &inv));
	zassert_within(inv.m[8], 1.0, 1e-15);
	zassert_true(at_h33_apply(&h, 100.0, -50.0, &x, &y));
	zassert_true(at_h33_apply(&inv, x, y, &x2, &y2));
	zassert_within(x2, 100.0, 1e-9);
	zassert_within(y2, -50.0, 1e-9);

	at_h33 sing = {{1, 2, 3, 2, 4, 6, 0, 0, 1}};

	zassert_false(at_h33_invert(&sing, &inv));
	at_h33 zero = {{0}};

	zassert_false(at_h33_invert(&zero, &inv));
	/* A point on the line at infinity. */
	at_h33 p = {{1, 0, 0, 0, 1, 0, 1, 0, 0}};

	zassert_false(at_h33_apply(&p, 0.0, 5.0, &x, &y));
}

ZTEST(geom, test_dlt)
{
	at_h33 h = h_ref(), est;
	double src[2 * 20], dst[2 * 20];
	uint32_t seed = 5U;

	for (int i = 0; i < 20; i++) {
		src[2 * i] = (tu_unif(&seed) - 0.5) * 3000.0;
		src[(2 * i) + 1] = (tu_unif(&seed) - 0.5) * 1700.0;
		(void)at_h33_apply(&h, src[2 * i], src[(2 * i) + 1], &dst[2 * i], &dst[(2 * i) + 1]);
	}
	/* Exact from 4 points, least squares from 20. */
	for (int n = 4; n <= 20; n += 16) {
		zassert_ok(at_h33_dlt(src, dst, n, &est));
		for (int i = 0; i < 9; i++) {
			zassert_within(est.m[i], h.m[i], 1e-9 * fmax(1.0, fabs(h.m[i])), "n %d m[%d]", n, i);
		}
	}
	/* Degenerate: too few points, all points on one line. */
	zassert_equal(at_h33_dlt(src, dst, 3, &est), -1);
	for (int i = 0; i < 6; i++) {
		src[2 * i] = (double)i;
		src[(2 * i) + 1] = 2.0 * i;
	}
	zassert_equal(at_h33_dlt(src, dst, 6, &est), -1);
	for (int i = 0; i < 6; i++) {
		src[2 * i] = 7.0;
		src[(2 * i) + 1] = 7.0;
	}
	zassert_equal(at_h33_dlt(src, dst, 6, &est), -1, "all points equal");
}

ZTEST(geom, test_image_screen_mapping)
{
	at_geom g;
	at_h33 to_cam;
	double u, v, x, y;

	ts_camera_h(&to_cam);
	zassert_ok(at_geom_init(&g, &lens_ref, &to_cam, 3000.0, 1690.0));

	/* The screen centre is on the optical axis: the image centre. */
	zassert_true(at_geom_to_image(&g, 0.0, 0.0, &u, &v));
	zassert_within(u, 639.5, 1e-9);
	zassert_within(v, 399.5, 1e-9);

	/* Round trip over the screen and beyond its edge. */
	for (int i = -8; i <= 8; i++) {
		for (int j = -5; j <= 5; j++) {
			double px = 200.0 * i;
			double py = 180.0 * j;

			zassert_true(at_geom_to_image(&g, px, py, &u, &v));
			int st = at_geom_to_screen(&g, u, v, &x, &y);

			zassert_within(x, px, 1e-7);
			zassert_within(y, py, 1e-7);
			bool on = (fabs(px) <= 1500.0) && (fabs(py) <= 845.0);

			zassert_equal(st, on ? AT_GEOM_ON_SCREEN : AT_GEOM_OFF_SCREEN, "(%g, %g)", px, py);
		}
	}
	/* About 2.5 mm per pixel at the centre (SPEC §2a). */
	zassert_true(at_geom_to_image(&g, 0.0, 0.0, &u, &v));
	zassert_equal(at_geom_to_screen(&g, u + 1.0, v, &x, &y), AT_GEOM_ON_SCREEN);
	zassert_within(x, 2.54, 0.01);

	/* Invalid set-ups. */
	at_lens bad = lens_ref;

	bad.f = 0.0;
	zassert_equal(at_geom_init(&g, &bad, &to_cam, 3000.0, 1690.0), -1);
	at_h33 sing = {{0}};

	zassert_equal(at_geom_init(&g, &lens_ref, &sing, 3000.0, 1690.0), -1);

	/* A pixel the distortion model cannot invert. */
	at_lens fold = {.f = 100.0, .cx = 0.0, .cy = 0.0, .k1 = -1.0};

	zassert_ok(at_geom_init(&g, &fold, &to_cam, 3000.0, 1690.0));
	zassert_equal(at_geom_to_screen(&g, 200.0, 0.0, &x, &y), AT_GEOM_FAIL);
	/* A screen point the homography sends to infinity. */
	at_h33 inf = {{1, 0, 0, 0, 1, 0, 1, 0, 1}};

	zassert_ok(at_geom_init(&g, &lens_ref, &inf, 3000.0, 1690.0));
	zassert_false(at_geom_to_image(&g, -1.0, 3.0, &u, &v));
}
