/*
 * Lane time from SYNC, SPEC F06 (the weapon firmware's scheme, S1 F12).
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

void at_ts_add(at_timesync *ts, int64_t local_us, uint32_t lane_us)
{
	int64_t lane = unwrap(ts, lane_us);

	if (ts->synced) {
		double err = (double)lane - (ts->ref_lane + (ts->slope * (double)(local_us - ts->ref_local)));

		if (fabs(err) > AT_TS_JUMP_US) {
			ts->n = 0U;
			ts->head = 0U;
			ts->resets++;
		}
	}
	ts->local[ts->head] = local_us;
	ts->lane[ts->head] = lane;
	ts->head = (uint8_t)((ts->head + 1U) % AT_TS_WINDOW);
	if (ts->n < AT_TS_WINDOW) {
		ts->n++;
	}
	ts->updates++;
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
