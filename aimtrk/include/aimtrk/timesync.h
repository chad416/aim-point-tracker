/*
 * Lane time from SYNC (SPEC F06): the same scheme as the weapon firmware
 * (S1 F12, app/src/timesync). Each SYNC is timestamped with the tracker's own
 * clock on reception, and
 *     lane = ref_lane + slope * (local - ref_local)
 * is fitted by least squares over the last AT_TS_WINDOW points: offset and
 * drift. The 32-bit lane clock is unwrapped to 64 bits; a jump of more than
 * AT_TS_JUMP_US (lane restart) restarts the fit; the drift is bounded to
 * +-AT_TS_MAX_DRIFT so a bad SYNC stream cannot skew every timestamp.
 */
#ifndef AIMTRK_TIMESYNC_H_
#define AIMTRK_TIMESYNC_H_

#include <stdbool.h>
#include <stdint.h>

#define AT_TS_WINDOW    32U
#define AT_TS_JUMP_US   10000
#define AT_TS_MAX_DRIFT 500e-6

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
	uint32_t updates, resets;
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
