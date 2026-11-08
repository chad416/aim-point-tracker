/*
 * aim_host: the tracker service (SPEC §4, §10).
 *
 * Takes camera frames (frames.h) and the lane's CAN traffic, runs aimtrk
 * (detect, geometry, match), and sends HIT, AIM and TRK_STATUS on CAN. Writes
 * an after-action record per shot (SPEC F12) and, optionally, a line per frame.
 *
 * Two transports:
 *   files   --can-in log --can-out log: everything is replayed on the
 *           tracker's own clock from the timestamps (frames: exposure start
 *           in the header; CAN: reception time), so a run is repeatable.
 *           Frame processing time (thread CPU time) is measured and added to the clock.
 *   socket  --can vcan0: SocketCAN, frames from a pipe, CLOCK_MONOTONIC. CAN
 *           frames carry their kernel reception time (canio.h), so SYNC is
 *           stamped when it arrived, not when the loop got to it.
 *
 * Usage: aim_host --frames FILE|- --calib calib.json --weapon NODE:SLOT [...]
 *                 (--can-in IN --can-out OUT | --can IFNAME [--can-log OUT])
 *                 [--slots N] [--aar aar.jsonl] [--frame-log frames.jsonl]
 *                 [--readout-us 2500] [--zero-in zero.txt]
 *                 [--zero NODE:X:Y:N --zero-out zero.txt] [--rt]
 *
 *   --rt    mlockall() and SCHED_FIFO for the live loop (needs the rights;
 *           a warning, not an error, without them). Off by default: the
 *           measurements in results/ were taken without it.
 */
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <malloc.h>
#include <math.h>
#include <poll.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "aimtrk/detect.h"
#include "aimtrk/geom.h"
#include "aimtrk/match.h"
#include "aimtrk/proto.h"
#include "aimtrk/timesync.h"
#include "aimtrk/zero.h"
#include "calibfile.h"
#include "canio.h"
#include "frames.h"

#define MAX_SPOTS    32
#define AIM_HIST     160 /* > 1.5 s of aim at 100 Hz */
#define AAR_MAX      64
#define DETAIL_MAX   16
#define FOREVER      INT64_MAX
#define AAR_BEFORE   1000000 /* aim trace from 1.0 s before the break ... */
#define AAR_AFTER    500000  /* ... to 0.5 s after (SPEC F12) */
#define SLOW_MS      3.0     /* P05 */
#define CAL_OLD_H    24
#define N_RESULTS    6    /* enum at_hit_result: OK .. DROPPED */
#define ZERO_MAX_SPREAD_MM 20.0 /* a zeroing group wider than this (RMS radius) is refused: not a steady hold */
#define RT_PRIORITY  50   /* --rt: SCHED_FIFO priority */

typedef struct {
	int64_t t;
	float x, y;
} aim_pt;

typedef struct {
	aim_pt p[AIM_HIST];
	int head, n;
} aim_hist;

typedef struct {
	bool used;
	at_mevent hit;
	float latency_ms;
	bool have_detail;
	at_shot_detail detail;
} aar_rec;

typedef struct {
	bool used;
	uint8_t node;
	at_shot_detail d;
} detail_rec;

typedef struct {
	/* options */
	const char *frames_path, *calib_path, *can_in, *can_out, *can_if, *can_log, *aar_path, *frame_log_path;
	const char *zero_in, *zero_out;
	bool rt;
	int64_t readout_us;
	int zero_node, zero_n;
	double zero_tx, zero_ty;
	/* state */
	at_match_cfg mcfg;
	at_match match;
	at_detect det;
	void *det_mem;
	bool det_ready;
	at_spot spots[MAX_SPOTS];
	at_geom geom;
	int64_t cal_created;
	at_timesync ts;
	canio can;
	frame_src fs;
	FILE *aar, *frame_log;
	aim_hist hist[AT_MAX_WEAPONS];
	aar_rec rec[AAR_MAX];
	detail_rec details[DETAIL_MAX];
	int detail_next;
	int64_t clock;            /* the tracker's clock, us */
	int64_t next_status_lane; /* lane time of the next TRK_STATUS */
	bool have_index;
	uint32_t last_index;
	int zero_got;
	/* per second */
	uint32_t sec_frames, sec_dropped;
	double sec_worst_ms;
	uint8_t faults;
	/* totals */
	uint32_t frames, dropped, no_sync_frames, aims, hits[N_RESULTS], hits_invalid, status_sent, can_in_frames;
	uint32_t bad_size_frames; /* frames whose size differs from the first one: skipped */
	uint32_t lost_at_status;  /* CAN frames not sent plus shots not taken, at the last TRK_STATUS */
	double proc_sum_ms, proc_max_ms, proc_wall_max_ms, ambient;
	int64_t p07_max_us;
	size_t heap_first_frame, heap_end;
} host;

static int64_t mono_us(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return ((int64_t)t.tv_sec * 1000000) + (t.tv_nsec / 1000);
}

/*
 * CPU time of this thread (P05). It leaves out the time the guest kernel ran
 * other threads, but not the time a hypervisor takes the virtual CPU away
 * while this thread is running (steal time on WSL2 and CI runners): that
 * still counts. The wall time of each frame is measured as well. On WSL2
 * this clock reads about 8 % more than CLOCK_MONOTONIC over the same frames
 * (IT09), so it is not even a lower bound of the wall time there; both are
 * reported, and neither gives a worst case on a virtual machine.
 */
static int64_t cpu_us(void)
{
	struct timespec t;

	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
	return ((int64_t)t.tv_sec * 1000000) + (t.tv_nsec / 1000);
}

/* Bytes in use on the heap (glibc). Measured after the first frame and at the end: no growth (IT09). */
static size_t heap_in_use(void)
{
	return mallinfo2().uordblks;
}

/* Output files get static buffers, so stdio does not allocate them on first use. */
static char aar_buf[1 << 16], frame_log_buf[1 << 16], zero_buf[256];

static int weapon_index(const host *h, uint8_t node)
{
	for (int i = 0; i < h->mcfg.n_weapons; i++) {
		if (h->mcfg.weapon[i].node == node) {
			return i;
		}
	}
	return -1;
}

static void send(host *h, uint16_t id, const uint8_t *d, int len)
{
	can_msg m = {.t_local_us = h->clock, .id = id, .len = (uint8_t)len};

	memcpy(m.data, d, (size_t)len);
	/* A refused frame is counted in h->can.tx_dropped and flagged in the next TRK_STATUS. */
	(void)canio_send(&h->can, &m);
}

static const char *result_name(uint8_t r)
{
	static const char *const n[N_RESULTS] = {"OK", "NO_SPOT", "OFF_SCREEN", "AMBIGUOUS", "SATURATED", "DROPPED"};

	return (r < N_RESULTS) ? n[r] : "?";
}

/* ---- after-action records (SPEC F12) ---------------------------------------------- */

static void write_aar(host *h, aar_rec *r)
{
	if (h->aar != NULL) {
		const at_mevent *e = &r->hit;
		int w = weapon_index(h, e->node);

		fprintf(h->aar,
			"{\"node\":%u,\"shot_no\":%u,\"t_break_us\":%" PRId64 ",\"k\":%" PRId64
			",\"hit\":{\"result\":\"%s\",\"x_mm\":%.3f,\"y_mm\":%.3f,\"quality\":%u,\"latency_ms\":%.2f},",
			e->node, e->shot_no, e->t_break_us, e->k, result_name(e->result), (double)e->x_mm,
			(double)e->y_mm, e->quality, (double)r->latency_ms);
		if (r->have_detail) {
			fprintf(h->aar,
				"\"detail\":{\"takeup_ms\":%u,\"squeeze_ms\":%u,\"jerk\":%u,\"muzzle_rate\":%u,"
				"\"cant\":%d,\"flinch\":%.1f},",
				r->detail.takeup_ms, r->detail.squeeze_ms, r->detail.jerk, r->detail.muzzle_rate,
				r->detail.cant, (double)r->detail.flinch_x10 / 10.0);
		} else {
			fprintf(h->aar, "\"detail\":null,");
		}
		fprintf(h->aar, "\"trace\":[");
		bool first = true;

		if (w >= 0) {
			const aim_hist *a = &h->hist[w];

			for (int i = 0; i < a->n; i++) {
				const aim_pt *p = &a->p[(a->head - a->n + i + AIM_HIST) % AIM_HIST];
				int64_t dt = p->t - e->t_break_us;

				if ((dt >= -AAR_BEFORE) && (dt <= AAR_AFTER)) {
					fprintf(h->aar, "%s[%.1f,%.2f,%.2f]", first ? "" : ",", (double)dt / 1000.0,
						(double)p->x, (double)p->y);
					first = false;
				}
			}
		}
		fprintf(h->aar, "]}\n");
	}
	r->used = false;
}

static void aar_poll(host *h, int64_t lane_now, bool flush)
{
	for (int i = 0; i < AAR_MAX; i++) {
		if (h->rec[i].used && (flush || (lane_now >= (h->rec[i].hit.t_break_us + AAR_AFTER + 30000)))) {
			write_aar(h, &h->rec[i]);
		}
	}
}

static void aar_add(host *h, const at_mevent *e, float latency_ms)
{
	int slot = -1;

	for (int i = 0; i < AAR_MAX; i++) {
		if (!h->rec[i].used) {
			slot = i;
			break;
		}
	}
	if (slot < 0) {
		/* Full: write the oldest record now (its trace may be short). */
		int oldest = 0;

		for (int i = 1; i < AAR_MAX; i++) {
			if (h->rec[i].hit.t_break_us < h->rec[oldest].hit.t_break_us) {
				oldest = i;
			}
		}
		write_aar(h, &h->rec[oldest]);
		slot = oldest;
	}
	aar_rec *r = &h->rec[slot];

	memset(r, 0, sizeof(*r));
	r->used = true;
	r->hit = *e;
	r->latency_ms = latency_ms;
	for (int i = 0; i < DETAIL_MAX; i++) {
		if (h->details[i].used && (h->details[i].node == e->node) && (h->details[i].d.shot_no == e->shot_no)) {
			r->have_detail = true;
			r->detail = h->details[i].d;
		}
	}
}

/* SHOT_DETAIL usually comes before the HIT; keep the last few and attach them either way. */
static void detail_add(host *h, uint8_t node, const at_shot_detail *d)
{
	for (int i = 0; i < AAR_MAX; i++) {
		if (h->rec[i].used && (h->rec[i].hit.node == node) && (h->rec[i].hit.shot_no == d->shot_no)) {
			h->rec[i].have_detail = true;
			h->rec[i].detail = *d;
		}
	}
	h->details[h->detail_next] = (detail_rec){true, node, *d};
	h->detail_next = (h->detail_next + 1) % DETAIL_MAX;
}

/* ---- match events ---------------------------------------------------------------------- */

static void handle_event(host *h, const at_mevent *e, int64_t lane_now)
{
	uint8_t d[8];
	int w = weapon_index(h, e->node);

	if (e->kind == AT_EV_AIM) {
		at_aim_msg a = {.frame_seq = (uint16_t)((uint64_t)e->k & 0xFFFFU), .x_mm = e->x_mm, .y_mm = e->y_mm};

		send(h, AT_CAN_ID(AT_FN_AIM, e->node), d, at_proto_aim(&a, d));
		if (w >= 0) {
			aim_hist *hs = &h->hist[w];

			hs->p[hs->head] = (aim_pt){at_match_pulse_mid(e->k), e->x_mm, e->y_mm};
			hs->head = (hs->head + 1) % AIM_HIST;
			hs->n = (hs->n < AIM_HIST) ? (hs->n + 1) : AIM_HIST;
		}
		h->aims++;
		return;
	}
	float latency = (float)(lane_now - e->t_break_us) / 1000.0F;
	at_hit_msg m = {.shot_no = e->shot_no, .x_mm = e->x_mm, .y_mm = e->y_mm, .result = e->result,
			.quality = e->quality, .latency_ms = latency};

	send(h, AT_CAN_ID(AT_FN_HIT, e->node), d, at_proto_hit(&m, d));
	if (e->result < N_RESULTS) {
		h->hits[e->result]++;
	} else {
		h->hits_invalid++; /* not an at_hit_result: never expected */
	}
	aar_add(h, e, latency);

	/* Zeroing session: OK hits of the zeroing weapon, in raw coordinates. */
	if ((e->node == h->zero_node) && (e->result == AT_HIT_OK) && (h->zero_got < h->zero_n)) {
		at_zero *z = at_match_zero(&h->match, e->node);
		double x = e->x_mm, y = e->y_mm;

		if (z->valid) {
			x += z->off_x;
			y += z->off_y;
		}
		at_zero_add(z, x, y, h->zero_tx, h->zero_ty);
		if (++h->zero_got == h->zero_n) {
			int rc = at_zero_finish(z, h->zero_n, ZERO_MAX_SPREAD_MM);

			fprintf(stderr, "zeroing node %d: %s, offset (%.2f, %.2f) mm, group %.2f mm\n", e->node,
				(rc == AT_ZERO_OK) ? "ok" : "refused", z->off_x, z->off_y, z->spread_mm);
			if ((rc == AT_ZERO_OK) && (h->zero_out != NULL)) {
				FILE *f = fopen(h->zero_out, "w");

				if (f != NULL) {
					(void)setvbuf(f, zero_buf, _IOFBF, sizeof(zero_buf));
					fprintf(f, "%d %.4f %.4f\n", e->node, z->off_x, z->off_y);
					fclose(f);
				}
			}
		}
	}
}

static void poll_match(host *h)
{
	at_mevent ev[32];
	int64_t lane = at_ts_lane(&h->ts, h->clock);
	int n;

	if (!h->ts.synced) {
		return;
	}
	do {
		n = at_match_poll(&h->match, lane, ev, 32);
		for (int i = 0; i < n; i++) {
			handle_event(h, &ev[i], lane);
		}
	} while (n > 0);
	aar_poll(h, lane, false);
}

/* Tracker-clock time at which polling the match can make progress next. */
static int64_t next_due(const host *h)
{
	const at_match *m = &h->match;

	if (!m->started || !h->ts.synced) {
		return FOREVER;
	}
	int64_t k = m->next_settle;
	const at_mframe *f = &m->ring[(uint64_t)k % AT_MATCH_RING];
	int64_t due = at_match_exposure_end(k) + m->cfg.settle_us;

	if (!(f->used && (f->k == k) && f->present) && (m->newest <= k)) {
		due += m->cfg.drop_wait_us;
	}
	return at_ts_local(&h->ts, due);
}

/* ---- status --------------------------------------------------------------------------- */

static void send_status(host *h)
{
	uint8_t d[8];
	double age_h = (h->cal_created > 0) ? ((double)(time(NULL) - h->cal_created) / 3600.0) : 0.0;
	at_status_msg s = {.fps = (float)h->sec_frames, .dropped = (uint8_t)((h->sec_dropped > 255U) ? 255U : h->sec_dropped),
			   .worst_ms = (float)h->sec_worst_ms, .ambient = (uint8_t)fmin(fmax(h->ambient, 0.0), 255.0),
			   .cal_age_h = (uint8_t)fmin(age_h, 255.0), .faults = h->faults};

	if (age_h > CAL_OLD_H) {
		s.faults |= AT_FAULT_CALIBRATION_OLD;
	}
	if (h->sec_dropped > 0U) {
		s.faults |= AT_FAULT_FRAMES_DROPPED;
	}
	if (h->sec_worst_ms > SLOW_MS) {
		s.faults |= AT_FAULT_SLOW;
	}
	if (!h->ts.synced) {
		s.faults |= AT_FAULT_NO_SYNC;
	}
	uint32_t lost = h->can.tx_dropped + h->match.stats.shots_lost;

	if (lost != h->lost_at_status) {
		s.faults |= AT_FAULT_CAN_LOSS; /* since the last TRK_STATUS */
		h->lost_at_status = lost;
	}
	send(h, AT_CAN_ID(AT_FN_TRK_STATUS, AT_TRACKER_NODE), d, at_proto_status(&s, d));
	h->status_sent++;
	h->sec_frames = 0U;
	h->sec_dropped = 0U;
	h->sec_worst_ms = 0.0;
	h->faults &= (uint8_t)~AT_FAULT_DETECT_OVERFLOW;
}

/* ---- inputs -------------------------------------------------------------------------- */

static void handle_can(host *h, const can_msg *c)
{
	uint8_t fn = AT_CAN_FN(c->id);
	uint8_t node = AT_CAN_NODE(c->id);
	int64_t lane = at_ts_lane(&h->ts, c->t_local_us);

	h->can_in_frames++;
	if ((fn == AT_FN_SYNC) && (node == 0U)) {
		uint32_t t;

		if (at_proto_sync(c->data, c->len, &t)) {
			at_ts_add(&h->ts, c->t_local_us, t);
			if (h->next_status_lane == 0) {
				h->next_status_lane = at_ts_lane(&h->ts, c->t_local_us) + 1000000;
			}
		}
	} else if (fn == AT_FN_SHOT_EVENT) {
		at_shot_event s;

		if (at_proto_shot_event(c->data, c->len, &s) && h->ts.synced) {
			uint32_t lost = h->match.stats.shots_lost;

			/* Refused: an unknown node (counted in match.unknown_node), or the shot table is full. */
			if ((at_match_shot(&h->match, node, s.shot_no, at_ts_unwrap_near(s.t_us, lane)) != 0) &&
			    (h->match.stats.shots_lost != lost)) {
				fprintf(stderr, "aim_host: shot %u of node %u lost: pending-shot table full\n", s.shot_no,
					node);
			}
		}
	} else if (fn == AT_FN_SHOT_DETAIL) {
		at_shot_detail s;

		if (at_proto_shot_detail(c->data, c->len, &s)) {
			detail_add(h, node, &s);
		}
	}
}

static int init_detect(host *h, uint16_t w, uint16_t ht)
{
	at_detect_cfg cfg;

	at_detect_cfg_default(&cfg, w, ht);
	size_t size = at_detect_mem_size(&cfg);

	h->det_mem = malloc(size); /* once, at start-up */
	if ((h->det_mem == NULL) || (at_detect_init(&h->det, &cfg, h->det_mem, size) != 0)) {
		return -1;
	}
	h->det_ready = true;
	return 0;
}

/* Process the buffered frame; t_ready is when it is available to the tracker. */
static void handle_frame(host *h, int64_t t_ready, bool live)
{
	const frame_hdr *hd = &h->fs.hdr;

	if (h->have_index && (hd->index != (h->last_index + 1U))) {
		uint32_t gap = hd->index - h->last_index - 1U;

		h->dropped += gap;
		h->sec_dropped += gap;
	}
	h->have_index = true;
	h->last_index = hd->index;
	if (!h->det_ready && (init_detect(h, hd->width, hd->height) != 0)) {
		fprintf(stderr, "aim_host: bad frame size %ux%u\n", hd->width, hd->height);
		exit(2);
	}
	if ((hd->width != h->det.cfg.width) || (hd->height != h->det.cfg.height)) {
		/* The detector is sized by the first frame. Another size is skipped: its exposure counts as dropped. */
		if (h->bad_size_frames++ == 0U) {
			fprintf(stderr, "aim_host: frame %u is %ux%u, not %ux%u: skipped\n", hd->index, hd->width,
				hd->height, h->det.cfg.width, h->det.cfg.height);
		}
		return;
	}
	if (!h->ts.synced) {
		h->no_sync_frames++;
		return;
	}
	int64_t lane_t0 = at_ts_lane(&h->ts, hd->t_local_us);
	int64_t k = (int64_t)llround((double)(lane_t0 - AT_EXPOSURE_START_US) / AT_FRAME_US);
	int64_t residual = lane_t0 - ((k * AT_FRAME_US) + AT_EXPOSURE_START_US);

	int64_t t0 = cpu_us();
	int64_t w0 = mono_us();
	int n = at_detect_frame(&h->det, h->fs.pixels, hd->width, h->spots, MAX_SPOTS);
	at_mspot ms[MAX_SPOTS];
	uint8_t dflags[MAX_SPOTS]; /* detect's own flags, for the frame log */
	float dsig[MAX_SPOTS];
	int nm = 0;

	for (int i = 0; i < n; i++) {
		double x, y;
		int st = at_geom_to_screen(&h->geom, h->spots[i].x, h->spots[i].y, &x, &y);

		if (st == AT_GEOM_FAIL) {
			continue;
		}
		dflags[nm] = h->spots[i].flags;
		dsig[nm] = h->spots[i].sig_major;
		ms[nm++] = (at_mspot){.x_mm = (float)x, .y_mm = (float)y, .snr = h->spots[i].snr,
				      .flags = (uint8_t)(((st == AT_GEOM_OFF_SCREEN) ? AT_MSPOT_OFF_SCREEN : 0U) |
							 (((h->spots[i].flags & AT_SPOT_RIM_FAILED) != 0U) ? AT_MSPOT_SATURATED
													  : 0U))};
	}
	int64_t proc_us = cpu_us() - t0;
	double proc_ms = (double)proc_us / 1000.0;
	double wall_ms = (double)(mono_us() - w0) / 1000.0;

	at_match_frame(&h->match, k, ms, nm);
	const at_detect_stats *st = at_detect_get_stats(&h->det);

	if (st->overflow != 0U) {
		h->faults |= AT_FAULT_DETECT_OVERFLOW;
	}
	h->ambient = st->bg_mean;
	h->frames++;
	h->sec_frames++;
	h->proc_sum_ms += proc_ms;
	h->proc_max_ms = fmax(h->proc_max_ms, proc_ms);
	h->proc_wall_max_ms = fmax(h->proc_wall_max_ms, wall_ms);
	if (h->frames == 1U) {
		h->heap_first_frame = heap_in_use();
	}
	h->sec_worst_ms = fmax(h->sec_worst_ms, proc_ms);
	if (llabs(residual) > h->p07_max_us) {
		h->p07_max_us = llabs(residual);
	}
	if (h->frame_log != NULL) {
		fprintf(h->frame_log,
			"{\"index\":%u,\"k\":%" PRId64 ",\"lane_t0_us\":%" PRId64 ",\"spots\":%d,"
			"\"proc_ms\":%.3f,\"wall_ms\":%.3f,\"sigma\":%.3f,"
			"\"rejected\":[%u,%u,%u,%u],\"overflow\":%u,\"spot_xy\":[",
			hd->index, k, lane_t0, nm, proc_ms, wall_ms, (double)st->sigma, st->rejected[0],
			st->rejected[1], st->rejected[2], st->rejected[3], st->overflow);
		for (int i = 0; i < nm; i++) {
			fprintf(h->frame_log, "%s[%.3f,%.3f,%.1f,%u,%u,%.2f]", (i > 0) ? "," : "", (double)ms[i].x_mm,
				(double)ms[i].y_mm, (double)ms[i].snr, ms[i].flags, dflags[i], (double)dsig[i]);
		}
		fprintf(h->frame_log, "]}\n");
	}
	/* The tracker's clock moves on by the processing time (file mode); live, it is real. */
	h->clock = live ? mono_us() : ((t_ready + proc_us > h->clock) ? (t_ready + proc_us) : h->clock);
}

/* ---- main loops -------------------------------------------------------------------- */

static void advance(host *h, int64_t t)
{
	if (t > h->clock) {
		h->clock = t;
	}
}

static int run_files(host *h)
{
	int rc = frames_next(&h->fs);

	while (true) {
		can_msg c;
		int64_t t_frame = (rc == 1) ? (h->fs.hdr.t_local_us + h->fs.hdr.exposure_us + h->readout_us) : FOREVER;
		int64_t t_can = canio_peek(&h->can, &c) ? c.t_local_us : FOREVER;
		int64_t t_due = next_due(h);
		int64_t t_status = (h->next_status_lane > 0) ? at_ts_local(&h->ts, h->next_status_lane) : FOREVER;

		if ((t_frame == FOREVER) && (t_can == FOREVER)) {
			break; /* inputs done: flush() settles what is left */
		}
		if (t_due <= h->clock) {
			t_due = FOREVER; /* nothing to do until another input arrives */
		}
		int64_t t = t_frame;

		t = (t_can < t) ? t_can : t;
		t = (t_due < t) ? t_due : t;
		t = (t_status < t) ? t_status : t;
		if (t == FOREVER) {
			break;
		}
		if (t == t_can) {
			advance(h, t);
			canio_pop(&h->can);
			handle_can(h, &c);
		} else if (t == t_frame) {
			advance(h, t);
			handle_frame(h, t, false);
			rc = frames_next(&h->fs);
			if (rc < 0) {
				fprintf(stderr, "aim_host: bad frame stream\n");
				return 2;
			}
		} else if (t == t_status) {
			advance(h, t);
			send_status(h);
			h->next_status_lane += 1000000;
			if ((rc != 1) && !h->can.have_next) {
				h->next_status_lane = 0; /* inputs done: no more status */
			}
		} else {
			advance(h, t);
		}
		poll_match(h);
	}
	return 0;
}

static int run_socket(host *h)
{
	int fd = fileno(h->fs.f);

	while (true) {
		struct pollfd p[2] = {{.fd = fd, .events = POLLIN}, {.fd = h->can.sock, .events = POLLIN}};
		int64_t now = mono_us();
		int64_t due = next_due(h);
		int64_t st = (h->next_status_lane > 0) ? at_ts_local(&h->ts, h->next_status_lane) : FOREVER;
		int64_t wake = (due < st) ? due : st;
		int timeout = (wake == FOREVER) ? 100 : (int)fmax(0.0, fmin(100.0, (double)(wake - now) / 1000.0));

		if (poll(p, 2, timeout) < 0) {
			if (errno == EINTR) {
				continue;
			}
			return 2;
		}
		h->clock = mono_us();
		if ((p[1].revents & POLLIN) != 0) {
			can_msg c;

			while (canio_recv(&h->can, h->clock, &c)) {
				handle_can(h, &c);
			}
		}
		if ((p[0].revents & (POLLIN | POLLHUP)) != 0) {
			int rc = frames_next(&h->fs);

			if (rc == 0) {
				break;
			}
			if (rc < 0) {
				return 2;
			}
			handle_frame(h, h->clock, true);
		}
		if ((h->next_status_lane > 0) && (at_ts_lane(&h->ts, h->clock) >= h->next_status_lane)) {
			send_status(h);
			h->next_status_lane += 1000000;
		}
		poll_match(h);
	}
	return 0;
}

static void flush(host *h)
{
	if (!h->ts.synced || !h->match.started) {
		return;
	}
	/* Settle every frame up to the last one received; then write the open records. */
	advance(h, at_ts_local(&h->ts, at_match_exposure_end(h->match.newest) + h->match.cfg.settle_us + 1));
	poll_match(h);
	aar_poll(h, at_ts_lane(&h->ts, h->clock), true);
}

static void summary(const host *h)
{
	const at_match_stats *m = &h->match.stats;

	printf("{\"frames\":%u,\"camera_dropped\":%u,\"frames_before_sync\":%u,\"frames_bad_size\":%u,\"can_in\":%u,"
	       "\"aim\":%u,\"hits\":{\"OK\":%u,\"NO_SPOT\":%u,\"OFF_SCREEN\":%u,\"AMBIGUOUS\":%u,\"SATURATED\":%u,"
	       "\"DROPPED\":%u,\"invalid\":%u},\"status_frames\":%u,"
	       "\"proc_ms\":{\"mean\":%.3f,\"max\":%.3f,\"wall_max\":%.3f},"
	       "\"heap_bytes\":{\"after_first_frame\":%zu,\"end\":%zu},\"frame_time_residual_max_us\":%" PRId64 ","
	       "\"drift_ppm\":%.2f,\"sync\":{\"received\":%u,\"rejected\":%u,\"resets\":%u},"
	       "\"match\":{\"frames\":%u,\"dropped\":%u,\"stray_spots\":%u,\"shots\":%u,\"late_shots\":%u,"
	       "\"ambiguous\":%u,\"unknown_node\":%u,\"reacquired\":%u,\"confirmed\":%u,\"shots_lost\":%u,"
	       "\"untracked_shots\":%u},"
	       "\"can_bad_lines\":%u,\"can_tx_dropped\":%u,\"can_user_stamps\":%u,\"can_rx_delay_max_us\":%" PRId64 "}\n",
	       h->frames, h->dropped, h->no_sync_frames, h->bad_size_frames, h->can_in_frames, h->aims, h->hits[0],
	       h->hits[1], h->hits[2], h->hits[3], h->hits[4], h->hits[5], h->hits_invalid, h->status_sent,
	       (h->frames > 0U) ? (h->proc_sum_ms / h->frames) : 0.0, h->proc_max_ms, h->proc_wall_max_ms,
	       h->heap_first_frame, h->heap_end, h->p07_max_us, at_ts_drift_ppm(&h->ts), h->ts.updates,
	       h->ts.rejected, h->ts.resets, m->frames, m->dropped, m->stray_spots, m->shots, m->late_shots,
	       m->ambiguous, m->unknown_node, m->reacquired, m->confirmed, m->shots_lost, m->untracked_shots,
	       h->can.bad_lines, h->can.tx_dropped, h->can.user_stamps, h->can.rx_delay_max_us);
}

static void usage(void)
{
	fprintf(stderr,
		"usage: aim_host --frames FILE|- --calib calib.json --weapon NODE:SLOT [--weapon ...]\n"
		"                (--can-in IN --can-out OUT | --can IFNAME [--can-log OUT])\n"
		"                [--slots N] [--aar FILE] [--frame-log FILE] [--readout-us US]\n"
		"                [--zero-in FILE] [--zero NODE:X:Y:N --zero-out FILE] [--rt]\n");
}

/* --rt: lock the memory (no page faults later) and run the loop under SCHED_FIFO. Best effort. */
static void go_realtime(void)
{
	struct sched_param sp = {.sched_priority = RT_PRIORITY};

	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
		fprintf(stderr, "aim_host: --rt: mlockall: %s (continuing)\n", strerror(errno));
	}
	if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
		fprintf(stderr, "aim_host: --rt: SCHED_FIFO: %s (continuing)\n", strerror(errno));
	}
}

static int load_zero(host *h)
{
	FILE *f = fopen(h->zero_in, "r");
	int node;
	double x, y;

	if (f == NULL) {
		return -1;
	}
	while (fscanf(f, "%d %lf %lf", &node, &x, &y) == 3) {
		at_zero *z = at_match_zero(&h->match, (uint8_t)node);

		if (z != NULL) {
			z->valid = true;
			z->off_x = x;
			z->off_y = y;
		}
	}
	fclose(f);
	return 0;
}

int main(int argc, char **argv)
{
	static host h;
	static const struct option opts[] = {
		{"frames", required_argument, NULL, 'f'},   {"calib", required_argument, NULL, 'c'},
		{"weapon", required_argument, NULL, 'w'},   {"slots", required_argument, NULL, 's'},
		{"can-in", required_argument, NULL, 'i'},   {"can-out", required_argument, NULL, 'o'},
		{"can", required_argument, NULL, 'n'},      {"can-log", required_argument, NULL, 'l'},
		{"aar", required_argument, NULL, 'a'},      {"frame-log", required_argument, NULL, 'F'},
		{"readout-us", required_argument, NULL, 'r'}, {"zero-in", required_argument, NULL, 'Z'},
		{"zero", required_argument, NULL, 'z'},     {"zero-out", required_argument, NULL, 'O'},
		{"rt", no_argument, NULL, 'R'},
		{NULL, 0, NULL, 0},
	};
	int c;

	at_match_cfg_default(&h.mcfg);
	h.readout_us = 2500;
	while ((c = getopt_long(argc, argv, "", opts, NULL)) != -1) {
		switch (c) {
		case 'f': h.frames_path = optarg; break;
		case 'c': h.calib_path = optarg; break;
		case 'w': {
			unsigned node, slot;

			if ((h.mcfg.n_weapons >= AT_MAX_WEAPONS) || (sscanf(optarg, "%u:%u", &node, &slot) != 2)) {
				usage();
				return 1;
			}
			h.mcfg.weapon[h.mcfg.n_weapons++] = (at_weapon_cfg){(uint8_t)node, (uint8_t)slot};
			break;
		}
		case 's': h.mcfg.n_slots = atoi(optarg); break;
		case 'i': h.can_in = optarg; break;
		case 'o': h.can_out = optarg; break;
		case 'n': h.can_if = optarg; break;
		case 'l': h.can_log = optarg; break;
		case 'a': h.aar_path = optarg; break;
		case 'F': h.frame_log_path = optarg; break;
		case 'r': h.readout_us = atoll(optarg); break;
		case 'Z': h.zero_in = optarg; break;
		case 'z':
			if (sscanf(optarg, "%d:%lf:%lf:%d", &h.zero_node, &h.zero_tx, &h.zero_ty, &h.zero_n) != 4) {
				usage();
				return 1;
			}
			break;
		case 'O': h.zero_out = optarg; break;
		case 'R': h.rt = true; break;
		default: usage(); return 1;
		}
	}
	if ((h.frames_path == NULL) || (h.calib_path == NULL) || (h.mcfg.n_weapons == 0) ||
	    ((h.can_if == NULL) && (h.can_out == NULL))) {
		usage();
		return 1;
	}
	char err[128];

	if (calib_load(h.calib_path, &h.geom, &h.cal_created, err, (int)sizeof(err)) != 0) {
		fprintf(stderr, "aim_host: calibration refused: %s\n", err);
		return 1;
	}
	if (at_match_init(&h.match, &h.mcfg) != 0) {
		fprintf(stderr, "aim_host: bad weapon/slot configuration (at least 2 slots, distinct slots and nodes)\n");
		return 1;
	}
	if ((h.zero_in != NULL) && (load_zero(&h) != 0)) {
		fprintf(stderr, "aim_host: cannot read %s\n", h.zero_in);
		return 1;
	}
	if ((h.zero_n > 0) && (at_match_zero(&h.match, (uint8_t)h.zero_node) != NULL)) {
		at_zero_begin(at_match_zero(&h.match, (uint8_t)h.zero_node));
	}
	at_ts_init(&h.ts);
	if (frames_open(&h.fs, h.frames_path, 4096U * 4096U) != 0) {
		fprintf(stderr, "aim_host: cannot open %s\n", h.frames_path);
		return 1;
	}
	int rc = (h.can_if != NULL) ? canio_open_socket(&h.can, h.can_if, h.can_log)
				    : canio_open_files(&h.can, h.can_in, h.can_out);

	if (rc != 0) {
		fprintf(stderr, "aim_host: CAN transport: %s\n", strerror(errno));
		return 1;
	}
	if (h.aar_path != NULL) {
		h.aar = fopen(h.aar_path, "w");
		if (h.aar != NULL) {
			(void)setvbuf(h.aar, aar_buf, _IOFBF, sizeof(aar_buf));
		}
	}
	if (h.frame_log_path != NULL) {
		h.frame_log = fopen(h.frame_log_path, "w");
		if (h.frame_log != NULL) {
			(void)setvbuf(h.frame_log, frame_log_buf, _IOFBF, sizeof(frame_log_buf));
		}
	}
	if (h.rt) {
		go_realtime(); /* MCL_FUTURE also locks the detector block allocated at the first frame */
	}
	rc = (h.can_if != NULL) ? run_socket(&h) : run_files(&h);
	flush(&h);
	h.heap_end = heap_in_use();
	summary(&h);
	canio_close(&h.can);
	frames_close(&h.fs);
	if (h.aar != NULL) {
		fclose(h.aar);
	}
	if (h.frame_log != NULL) {
		fclose(h.frame_log);
	}
	free(h.det_mem);
	return rc;
}
