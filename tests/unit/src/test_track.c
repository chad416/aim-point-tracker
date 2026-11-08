/*
 * SPEC F07, §6.6: constant-velocity aim track, prediction, loss and restart.
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "aimtrk/track.h"

ZTEST_SUITE(track, NULL, NULL, NULL, NULL, NULL);

ZTEST(track, test_follows_constant_velocity)
{
	at_track_cfg c;
	at_track t;
	double x, y;

	at_track_cfg_default(&c);
	at_track_reset(&t);
	zassert_false(at_track_predict(&t, &c, 0, &x, &y), "no track yet");

	/* Aim sweeping at (200, -100) mm/s, updated at 50 Hz. */
	for (int i = 0; i < 50; i++) {
		int64_t tu = (int64_t)i * 20000;

		at_track_update(&t, &c, tu, 100.0 + (0.2 * i * 20.0), 50.0 - (0.1 * i * 20.0));
	}
	zassert_true(at_track_predict(&t, &c, 49 * 20000 + 10000, &x, &y));
	zassert_within(x, 100.0 + (200.0 * 0.99), 0.5, "%f", x);
	zassert_within(y, 50.0 - (100.0 * 0.99), 0.5, "%f", y);
	zassert_within(t.vx, 200.0, 2.0);
	zassert_within(t.vy, -100.0, 2.0);
	zassert_equal(t.updates, 50U);
}

ZTEST(track, test_lost_and_restart)
{
	at_track_cfg c;
	at_track t;
	double x, y;

	at_track_cfg_default(&c);
	at_track_reset(&t);
	at_track_update(&t, &c, 0, 10.0, 20.0);
	at_track_update(&t, &c, 20000, 12.0, 20.0);
	zassert_true(at_track_predict(&t, &c, 500000, &x, &y), "0.48 s after the last spot: still there");
	zassert_false(at_track_predict(&t, &c, 520001, &x, &y), "more than 0.5 s: lost");

	/* The next spot restarts it at that spot, without the old velocity. */
	at_track_update(&t, &c, 900000, -300.0, 400.0);
	zassert_equal(t.restarts, 1U);
	zassert_equal(t.updates, 1U);
	zassert_true(at_track_predict(&t, &c, 910000, &x, &y));
	zassert_within(x, -300.0, 1e-9);
	zassert_within(y, 400.0, 1e-9);

	/* An update at the same or an earlier time is ignored: no division by zero, the velocity is kept. */
	at_track_update(&t, &c, 920000, -296.0, 400.0);
	double vx = t.vx;

	zassert_true(vx > 0.0);
	at_track_update(&t, &c, 920000, -310.0, 400.0);
	at_track_update(&t, &c, 910000, -320.0, 400.0);
	zassert_equal(t.ignored, 2U);
	zassert_equal(t.restarts, 1U, "not a restart");
	zassert_equal(t.updates, 2U);
	zassert_within(t.vx, vx, 1e-12);
	zassert_true(t.x > -300.0, "position not moved by the ignored updates");
}
