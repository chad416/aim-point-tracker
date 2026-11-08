/*
 * CAN frames of the tracker, SPEC §7. Bit layout as protocol/tracker.dbc.
 */
#include <math.h>
#include <string.h>

#include "aimtrk/proto.h"

/* Put the low `bits` of v at little-endian bit position `start`. */
static void put(uint8_t *d, int start, int bits, uint64_t v)
{
	for (int i = 0; i < bits; i++) {
		int b = start + i;

		if (((v >> i) & 1U) != 0U) {
			d[b / 8] |= (uint8_t)(1U << (b % 8));
		} else {
			d[b / 8] &= (uint8_t)~(1U << (b % 8));
		}
	}
}

static uint64_t get(const uint8_t *d, int start, int bits)
{
	uint64_t v = 0;

	for (int i = 0; i < bits; i++) {
		int b = start + i;

		v |= (uint64_t)((d[b / 8] >> (b % 8)) & 1U) << i;
	}
	return v;
}

static int64_t sign_extend(uint64_t v, int bits)
{
	uint64_t m = (uint64_t)1 << (bits - 1);

	return (int64_t)((v ^ m) - m);
}

/* Round to the nearest step and saturate to [lo, hi] raw counts. */
static int64_t raw(double value, double step, int64_t lo, int64_t hi)
{
	double r = floor((value / step) + 0.5);

	if (r < (double)lo) {
		return lo;
	}
	if (r > (double)hi) {
		return hi;
	}
	return (int64_t)r;
}

#define MM_LO (-8192)
#define MM_HI 8191

int at_proto_hit(const at_hit_msg *m, uint8_t out[8])
{
	memset(out, 0, 8);
	put(out, 0, 16, m->shot_no);
	put(out, 16, 14, (uint64_t)raw(m->x_mm, 0.25, MM_LO, MM_HI));
	put(out, 30, 14, (uint64_t)raw(m->y_mm, 0.25, MM_LO, MM_HI));
	put(out, 44, 4, m->result);
	put(out, 48, 8, m->quality);
	put(out, 56, 8, (uint64_t)raw(m->latency_ms, 0.1, 0, 255));
	return 8;
}

int at_proto_aim(const at_aim_msg *m, uint8_t out[8])
{
	memset(out, 0, 8);
	put(out, 0, 16, m->frame_seq);
	put(out, 16, 14, (uint64_t)raw(m->x_mm, 0.25, MM_LO, MM_HI));
	put(out, 30, 14, (uint64_t)raw(m->y_mm, 0.25, MM_LO, MM_HI));
	return 6;
}

int at_proto_status(const at_status_msg *m, uint8_t out[8])
{
	memset(out, 0, 8);
	put(out, 0, 16, (uint64_t)raw(m->fps, 0.1, 0, 65535));
	put(out, 16, 8, m->dropped);
	put(out, 24, 8, (uint64_t)raw(m->worst_ms, 0.1, 0, 255));
	put(out, 32, 8, m->ambient);
	put(out, 40, 8, m->cal_age_h);
	put(out, 48, 8, m->faults);
	return 7;
}

bool at_proto_sync(const uint8_t *d, int len, uint32_t *lane_time_us)
{
	if (len != 5) {
		return false;
	}
	*lane_time_us = (uint32_t)get(d, 8, 32);
	return true;
}

bool at_proto_shot_event(const uint8_t *d, int len, at_shot_event *m)
{
	if (len != 8) {
		return false;
	}
	m->shot_no = (uint16_t)get(d, 0, 16);
	m->t_us = (uint32_t)get(d, 16, 32);
	m->mode = (uint8_t)get(d, 48, 2);
	m->chambered = get(d, 50, 1) != 0U;
	m->rounds_left = (uint8_t)get(d, 56, 8);
	return true;
}

bool at_proto_shot_detail(const uint8_t *d, int len, at_shot_detail *m)
{
	if (len != 8) {
		return false;
	}
	m->shot_no = (uint16_t)get(d, 0, 16);
	m->takeup_ms = (uint16_t)(get(d, 16, 8) * 4U);
	m->squeeze_ms = (uint16_t)(get(d, 24, 8) * 4U);
	m->jerk = (uint8_t)get(d, 32, 8);
	m->muzzle_rate = (uint8_t)get(d, 40, 8);
	m->cant = (int8_t)sign_extend(get(d, 48, 8), 8);
	m->flinch_x10 = (int8_t)sign_extend(get(d, 56, 8), 8);
	return true;
}

bool at_proto_hit_decode(const uint8_t *d, int len, at_hit_msg *m)
{
	if (len != 8) {
		return false;
	}
	m->shot_no = (uint16_t)get(d, 0, 16);
	m->x_mm = (float)sign_extend(get(d, 16, 14), 14) * 0.25F;
	m->y_mm = (float)sign_extend(get(d, 30, 14), 14) * 0.25F;
	m->result = (uint8_t)get(d, 44, 4);
	m->quality = (uint8_t)get(d, 48, 8);
	m->latency_ms = (float)get(d, 56, 8) * 0.1F;
	return true;
}
