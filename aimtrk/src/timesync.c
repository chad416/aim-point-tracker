/*
 * Lane time from SYNC, SPEC F06 (the weapon firmware's scheme, its F12).
 */
#include <math.h>
#include <string.h>

#include "aimtrk/timesync.h"

#define LANE_WRAP (((int64_t)1) << 32)

void at_ts_init(at_timesync *ts)
{
	memset(ts, 0, sizeof(*ts));
	ts->slope = 1.0;
}

static int64_t unwrap(at_timesync *ts, uint32_t lane_us)
{
	if ((ts->updates > 0U) && (lane_us < ts->last_lane32) &&
	    ((ts->last_lane32 - lane_us) > 0x80000000U)) {
		ts->lane_epoch += LANE_WRAP;
	}
	ts->last_lane32 = lane_us;
	return ts->lane_epoch + (int64_t)lane_us;
}

static void fit(at_timesync *ts)
{
	/* Centre on the newest point to keep the doubles well conditioned. */
	uint8_t newest = (uint8_t)((ts->head + AT_TS_WINDOW - 1U) % AT_TS_WINDOW);
	int64_t l0 = ts->local[newest];
	int64_t n0 = ts->lane[newest];
	double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;

	for (uint8_t i = 0U; i < ts->n; i++) {
		sx += (double)(ts->local[i] - l0);
		sy += (double)(ts->lane[i] - n0);
	}
	double mx = sx / ts->n;
	double my = sy / ts->n;

	for (uint8_t i = 0U; i < ts->n; i++) {
		double dx = (double)(ts->local[i] - l0) - mx;
		double dy = (double)(ts->lane[i] - n0) - my;

		sxx += dx * dx;
		sxy += dx * dy;
	}
	ts->slope = (sxx > 0.0) ? (sxy / sxx) : 1.0;
	ts->slope = fmin(fmax(ts->slope, 1.0 - AT_TS_MAX_DRIFT), 1.0 + AT_TS_MAX_DRIFT);
	ts->ref_local = l0;
	ts->ref_lane = (double)n0 + my - (ts->slope * mx);
	ts->synced = true;
}

static void push(at_timesync *ts, int64_t local_us, int64_t lane)
{
	ts->local[ts->head] = local_us;
	ts->lane[ts->head] = lane;
	ts->head = (uint8_t)((ts->head + 1U) % AT_TS_WINDOW);
	if (ts->n < AT_TS_WINDOW) {
		ts->n++;
	}
}

/* The held-out SYNCs agree with each other: they describe one new lane-time offset. */
static bool pending_consistent(const at_timesync *ts)
{
	double lo = ts->pend_err[0];
	double hi = ts->pend_err[0];

	for (uint8_t i = 1U; i < ts->n_pend; i++) {
		lo = fmin(lo, ts->pend_err[i]);
		hi = fmax(hi, ts->pend_err[i]);
	}
	return (hi - lo) <= (double)AT_TS_GATE_US;
}

void at_ts_add(at_timesync *ts, int64_t local_us, uint32_t lane_us)
{
	int64_t lane = unwrap(ts, lane_us);

	ts->updates++;
	if (ts->synced) {
		double err = (double)lane - (ts->ref_lane + (ts->slope * (double)(local_us - ts->ref_local)));

		if (!(fabs(err) <= (double)AT_TS_GATE_US)) {
			/* An outlier (a late reception stamp), or one of the first SYNCs after a step. */
			ts->rejected++;
			ts->pend_local[ts->n_pend] = local_us;
			ts->pend_lane[ts->n_pend] = lane;
			ts->pend_err[ts->n_pend] = err;
			ts->n_pend++;
			if (ts->n_pend < AT_TS_STEP_N) {
				return;
			}
			if (!pending_consistent(ts)) {
				/* Scattered, not a step: forget the oldest and keep waiting. */
				for (uint8_t i = 1U; i < AT_TS_STEP_N; i++) {
					ts->pend_local[i - 1U] = ts->pend_local[i];
					ts->pend_lane[i - 1U] = ts->pend_lane[i];
					ts->pend_err[i - 1U] = ts->pend_err[i];
				}
				ts->n_pend--;
				return;
			}
			/* A step of the lane time: restart the fit on the held-out SYNCs. */
			ts->n = 0U;
			ts->head = 0U;
			ts->resets++;
			for (uint8_t i = 0U; i < ts->n_pend; i++) {
				push(ts, ts->pend_local[i], ts->pend_lane[i]);
			}
			ts->n_pend = 0U;
			fit(ts);
			return;
		}
	}
	ts->n_pend = 0U;
	push(ts, local_us, lane);
	fit(ts);
}

int64_t at_ts_lane(const at_timesync *ts, int64_t local_us)
{
	if (!ts->synced) {
		return local_us;
	}
	return (int64_t)floor(ts->ref_lane + (ts->slope * (double)(local_us - ts->ref_local)) + 0.5);
}

int64_t at_ts_local(const at_timesync *ts, int64_t lane_us)
{
	if (!ts->synced) {
		return lane_us;
	}
	return ts->ref_local + (int64_t)floor((((double)lane_us - ts->ref_lane) / ts->slope) + 0.5);
}

int64_t at_ts_unwrap_near(uint32_t lane32, int64_t near_lane_us)
{
	int64_t base = near_lane_us - (int64_t)(uint32_t)near_lane_us; /* multiple of 2^32 */
	int64_t t = base + (int64_t)lane32;

	if ((t - near_lane_us) > (LANE_WRAP / 2)) {
		t -= LANE_WRAP;
	} else if ((near_lane_us - t) > (LANE_WRAP / 2)) {
		t += LANE_WRAP;
	}
	return t;
}

double at_ts_drift_ppm(const at_timesync *ts)
{
	return (ts->slope - 1.0) * 1e6;
}
