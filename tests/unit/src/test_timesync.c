/*
 * SPEC F06: lane time from SYNC (offset, drift, wrap, restart, drift bound,
 * inverse, unwrapping a 32-bit stamp). The same scheme as S1's timesync suite.
 */
#include <math.h>
#include <stdlib.h>
#include <zephyr/ztest.h>

#include "aimtrk/timesync.h"
#include "test_util.h"

ZTEST_SUITE(timesync, NULL, NULL, NULL, NULL, NULL);

/* SYNC every 10 ms; the tracker clock runs at (1 + ppm) against the lane, with reception jitter. */
static void feed(at_timesync *ts, int n, int64_t local0, int64_t lane0, double ppm, double jitter_us,
		 uint32_t *seed)
{
	for (int i = 0; i < n; i++) {
		int64_t lane = lane0 + ((int64_t)i * 10000);
		int64_t local = local0 + (int64_t)llround((double)(lane - lane0) * (1.0 + (ppm * 1e-6)));

		if (jitter_us > 0.0) {
			local += (int64_t)llround(fabs(tu_randn(seed)) * jitter_us);
		}
		at_ts_add(ts, local, (uint32_t)lane);
	}
}

ZTEST(timesync, test_offset_and_drift)
{
	at_timesync ts;
	uint32_t seed = 9U;

	at_ts_init(&ts);
	zassert_equal(at_ts_lane(&ts, 1234), 1234, "before SYNC: local time");
	/* Tracker clock 200 ppm fast, 5 us reception jitter (one-sided: frames arrive late). */
	feed(&ts, 100, 123456789, 50000000, 200.0, 5.0, &seed);
	TC_PRINT("drift %.1f ppm\n", at_ts_drift_ppm(&ts));
	zassert_within(at_ts_drift_ppm(&ts), -200.0, 15.0, "lane/local slope = 1/(1+200 ppm)");

	/* Lane time of a local instant 5 ms after the last SYNC: within 50 us. */
	int64_t lane_true = 50000000 + (99 * 10000) + 5000;
	int64_t local = 123456789 + llround((double)(lane_true - 50000000) * 1.0002);
	int64_t err = at_ts_lane(&ts, local) - lane_true;

	TC_PRINT("lane time error %lld us\n", (long long)err);
	zassert_true(llabs(err) <= 50, "%lld", (long long)err);
	/* The inverse round-trips. */
	zassert_true(llabs(at_ts_local(&ts, at_ts_lane(&ts, local)) - local) <= 1);
}

ZTEST(timesync, test_wrap_restart_and_bound)
{
	at_timesync ts;
	uint32_t seed = 4U;

	at_ts_init(&ts);
	/* Across the 32-bit wrap of the lane clock. */
	feed(&ts, 40, 1000000, 4294967296LL - 200000, 0.0, 0.0, &seed);
	zassert_within((double)at_ts_lane(&ts, 1000000 + 390000), 4294967296.0 + 190000.0, 2.0);
	zassert_equal(ts.resets, 0U);

	/* The lane restarts: lane time jumps back; the fit starts again. */
	feed(&ts, 5, 2000000, 1000, 0.0, 0.0, &seed);
	zassert_equal(ts.resets, 1U);

	/* A SYNC stream with an absurd slope (2 %): the drift is bounded to 500 ppm. */
	at_ts_init(&ts);
	feed(&ts, 32, 0, 0, 20000.0, 0.0, &seed);
	zassert_within(at_ts_drift_ppm(&ts), -500.0, 1e-6);
	at_ts_init(&ts);
	feed(&ts, 32, 0, 0, -20000.0, 0.0, &seed);
	zassert_within(at_ts_drift_ppm(&ts), 500.0, 1e-6);

	/* Before any SYNC the inverse is the identity too. */
	at_ts_init(&ts);
	zassert_equal(at_ts_local(&ts, 77), 77);
}

ZTEST(timesync, test_unwrap_near)
{
	int64_t near = (3LL << 32) + 100;

	zassert_equal(at_ts_unwrap_near(50, near), (3LL << 32) + 50);
	/* A stamp just before the wrap, seen just after it. */
	zassert_equal(at_ts_unwrap_near(0xFFFFFF00U, near), (3LL << 32) - 256);
	/* A stamp just after the wrap, seen just before it. */
	zassert_equal(at_ts_unwrap_near(10, (4LL << 32) - 20), (4LL << 32) + 10);
	zassert_equal(at_ts_unwrap_near(5, -100), 5);
}
