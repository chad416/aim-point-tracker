/*
 * Lane time from SYNC (SPEC F06): the same scheme as the weapon firmware's
 * timesync (trainer-weapon-firmware-twin, F12). Each SYNC is timestamped with
 * the tracker's own clock on reception, and
 *     lane = ref_lane + slope * (local - ref_local)
 * is fitted by least squares over the last AT_TS_WINDOW points: offset and
 * drift. The 32-bit lane clock is unwrapped to 64 bits, and the drift is
 * bounded to +-AT_TS_MAX_DRIFT so a bad SYNC stream cannot skew every
 * timestamp.
 *
 * Outliers: a SYNC whose lane time is more than AT_TS_GATE_US from the
 * current fit is held out (a late reception stamp would otherwise pull the
 * whole fit). Only when AT_TS_STEP_N SYNCs in a row disagree with the fit,
 * and agree with each other to AT_TS_GATE_US, has the lane time really
 * stepped (a lane restart, a long gap): the fit restarts on them.
 */
#ifndef AIMTRK_TIMESYNC_H_
#define AIMTRK_TIMESYNC_H_

#include <stdbool.h>
#include <stdint.h>

#define AT_TS_WINDOW    32U
#define AT_TS_MAX_DRIFT 500e-6
#define AT_TS_GATE_US   200 /* SYNC further than this from the fit: held out */
#define AT_TS_STEP_N    4U  /* this many in a row, consistent with each other: a step, the fit restarts */

typedef struct {
	int64_t local[AT_TS_WINDOW];
	int64_t lane[AT_TS_WINDOW]; /* unwrapped */
	uint8_t n, head;
	uint32_t last_lane32;
	int64_t lane_epoch;
	bool synced;
	int64_t ref_local;
	double ref_lane;
	double slope;
	/* SYNCs held out in a row (candidates for a step) */
	int64_t pend_local[AT_TS_STEP_N];
	int64_t pend_lane[AT_TS_STEP_N];
	double pend_err[AT_TS_STEP_N];
	uint8_t n_pend;
	uint32_t updates;  /* SYNCs received */
	uint32_t resets;   /* fit restarted on a step */
	uint32_t rejected; /* SYNCs held out as outliers (those that later formed a step included) */
} at_timesync;

void at_ts_init(at_timesync *ts);

/* One SYNC: the tracker's clock at reception (us) and the 32-bit lane time it carried. */
void at_ts_add(at_timesync *ts, int64_t local_us, uint32_t lane_us);

/* Unwrapped lane time (us) for a local time; the local time itself before the first SYNC. */
int64_t at_ts_lane(const at_timesync *ts, int64_t local_us);

/* Local time for an unwrapped lane time (the inverse). */
int64_t at_ts_local(const at_timesync *ts, int64_t lane_us);

/* Unwrap a 32-bit lane time (e.g. a SHOT_EVENT's) to the 64-bit time nearest to `near_lane_us`. */
int64_t at_ts_unwrap_near(uint32_t lane32, int64_t near_lane_us);

double at_ts_drift_ppm(const at_timesync *ts);

#endif /* AIMTRK_TIMESYNC_H_ */
