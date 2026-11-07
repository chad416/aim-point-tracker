/*
 * SPEC F13: zeroing offset, its use, and refusal of a bad group.
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "aimtrk/zero.h"

ZTEST_SUITE(zero, NULL, NULL, NULL, NULL, NULL);

ZTEST(zero, test_offset_and_apply)
{
	at_zero z;
	double x = 100.0, y = 50.0;

	at_zero_reset(&z);
	at_zero_apply(&z, &x, &y);
	zassert_equal(x, 100.0, "no offset before zeroing");

	/* Five hits around (10, -5) mm from the target at (200, 300). */
	static const double d[5][2] = {{11, -4}, {9, -6}, {10, -5}, {12, -5}, {8, -5}};

	at_zero_begin(&z);
	for (int i = 0; i < 5; i++) {
		at_zero_add(&z, 200.0 + d[i][0], 300.0 + d[i][1], 200.0, 300.0);
	}
	zassert_equal(at_zero_finish(&z, 5, 20.0), AT_ZERO_OK);
	zassert_true(z.valid);
	zassert_within(z.off_x, 10.0, 1e-12);
	zassert_within(z.off_y, -5.0, 1e-12);
	zassert_within(z.spread_mm, sqrt(12.0 / 5.0), 1e-9); /* x: 1+1+0+4+4, y: 1+1+0+0+0 */
	zassert_equal(z.shots, 5);
	at_zero_apply(&z, &x, &y);
	zassert_within(x, 90.0, 1e-12);
	zassert_within(y, 55.0, 1e-12);
}

ZTEST(zero, test_refused_keeps_old_offset)
{
	at_zero z;

	at_zero_reset(&z);
	at_zero_begin(&z);
	for (int i = 0; i < 5; i++) {
		at_zero_add(&z, 3.0, 4.0, 0.0, 0.0);
	}
	zassert_equal(at_zero_finish(&z, 5, 20.0), AT_ZERO_OK);

	/* Too few shots. */
	at_zero_begin(&z);
	at_zero_add(&z, 50.0, 50.0, 0.0, 0.0);
	zassert_equal(at_zero_finish(&z, 5, 20.0), AT_ZERO_TOO_FEW);
	/* A scattered group (spread 100 mm). */
	at_zero_begin(&z);
	for (int i = 0; i < 6; i++) {
		at_zero_add(&z, (i % 2) ? 100.0 : -100.0, 0.0, 0.0, 0.0);
	}
	zassert_equal(at_zero_finish(&z, 5, 20.0), AT_ZERO_SCATTERED);
	zassert_within(z.off_x, 3.0, 1e-12, "old offset kept");
	zassert_within(z.off_y, 4.0, 1e-12);

	/* No shots at all. */
	at_zero_begin(&z);
	zassert_equal(at_zero_finish(&z, 0, 20.0), AT_ZERO_TOO_FEW);
}
