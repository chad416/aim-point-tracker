/*
 * Which weapon made which spot, SPEC F07-F10, §6.5.
 */
#include <math.h>
#include <string.h>

#include "aimtrk/match.h"
#include "aimtrk/proto.h"

#define MAX_EMITTERS (AT_MAX_WEAPONS + 4) /* aim pulses plus a few shot pulses */
#define NONE         (-1)

enum { EM_AIM, EM_SHOT };

typedef struct {
	int weapon;  /* index into cfg.weapon */
	int kind;    /* EM_AIM / EM_SHOT */
	int shot;    /* index into m->shot for EM_SHOT */
	bool has_pred;
	double px, py;
} emitter;

void at_match_cfg_default(at_match_cfg *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->n_slots = 2;
	cfg->gate_mm = 150.0;
	cfg->ambiguity_mm = 20.0;
	cfg->settle_us = 5000;
	cfg->drop_wait_us = 30000;
	at_track_cfg_default(&cfg->track);
}

int at_match_init(at_match *m, const at_match_cfg *cfg)
{
	if ((cfg->n_slots < 2) || (cfg->n_weapons < 0) || (cfg->n_weapons > AT_MAX_WEAPONS) ||
	    (cfg->gate_mm <= 0.0)) {
		return -1;
	}
	for (int i = 0; i < cfg->n_weapons; i++) {
		if ((cfg->weapon[i].slot >= cfg->n_slots) || (cfg->weapon[i].node == 0U) ||
		    (cfg->weapon[i].node > 127U)) {
			return -1;
		}
		for (int j = 0; j < i; j++) {
			if ((cfg->weapon[j].slot == cfg->weapon[i].slot) ||
			    (cfg->weapon[j].node == cfg->weapon[i].node)) {
				return -1;
			}
		}
	}
	memset(m, 0, sizeof(*m));
	m->cfg = *cfg;
	for (int i = 0; i < AT_MAX_WEAPONS; i++) {
		at_track_reset(&m->track[i]);
		at_zero_reset(&m->zero[i]);
	}
	return 0;
}

static int64_t ceil_div(int64_t a, int64_t b)
{
	return (a >= 0) ? ((a + b - 1) / b) : -((-a) / b);
}

int64_t at_match_shot_frame(int64_t t_break_us)
{
	return ceil_div(t_break_us + AT_STROBE_MARGIN_US - AT_PULSE_START_US, AT_FRAME_US);
}

int64_t at_match_exposure_end(int64_t k)
{
	return (k * AT_FRAME_US) + AT_EXPOSURE_START_US + AT_EXPOSURE_US;
}

static int slot_of(const at_match *m, int64_t k)
{
	int64_t s = k % m->cfg.n_slots;

	return (int)((s < 0) ? (s + m->cfg.n_slots) : s);
}

static int weapon_index(const at_match *m, uint8_t node)
{
	for (int i = 0; i < m->cfg.n_weapons; i++) {
		if (m->cfg.weapon[i].node == node) {
			return i;
		}
	}
	return NONE;
}

at_zero *at_match_zero(at_match *m, uint8_t node)
{
	int i = weapon_index(m, node);

	return (i == NONE) ? NULL : &m->zero[i];
}

const at_track *at_match_track(const at_match *m, uint8_t node)
{
	int i = weapon_index(m, node);

	return (i == NONE) ? NULL : &m->track[i];
}

void at_match_frame(at_match *m, int64_t k, const at_mspot *spots, int n)
{
	if (!m->started) {
		m->started = true;
		m->next_settle = k;
		m->newest = k;
	}
	m->stats.frames++;
	if (k > m->newest) {
		m->newest = k;
	}
	if (k < m->next_settle) {
		return; /* arrived after its frame was settled (as dropped) */
	}
	at_mframe *f = &m->ring[(uint64_t)k % AT_MATCH_RING];

	memset(f, 0, sizeof(*f));
	f->used = true;
	f->k = k;
	f->present = true;
	/* Keep the AT_MATCH_MAX_SPOTS strongest (insertion by SNR). */
	for (int i = 0; i < n; i++) {
		int pos = f->n;

		while ((pos > 0) && (spots[i].snr > f->s[pos - 1].snr)) {
			pos--;
		}
		if (pos >= AT_MATCH_MAX_SPOTS) {
			continue;
		}
		int last = (f->n < AT_MATCH_MAX_SPOTS) ? f->n : (AT_MATCH_MAX_SPOTS - 1);

		for (int j = last; j > pos; j--) {
			f->s[j] = f->s[j - 1];
		}
		f->s[pos] = spots[i];
		if (f->n < AT_MATCH_MAX_SPOTS) {
			f->n++;
		}
	}
}

int at_match_shot(at_match *m, uint8_t node, uint16_t shot_no, int64_t t_break_us)
{
	if (weapon_index(m, node) == NONE) {
		m->stats.unknown_node++;
		return -1;
	}
	for (int i = 0; i < AT_MATCH_SHOTS; i++) {
		if (!m->shot[i].active) {
			m->shot[i] = (at_mshot){true, node, shot_no, t_break_us, at_match_shot_frame(t_break_us)};
			return 0;
		}
	}
	return -1; /* table full: the shot is lost (counted by the caller) */
}

static at_mframe *frame_of(at_match *m, int64_t k)
{
	at_mframe *f = &m->ring[(uint64_t)k % AT_MATCH_RING];

	return (f->used && (f->k == k) && f->present) ? f : NULL;
}

static double spot_cost(const at_match *m, const emitter *e, const at_mspot *s)
{
	if (!e->has_pred) {
		/* No track yet: any spot will do; prefer the strongest. */
		return (0.5 * m->cfg.gate_mm) - (1e-3 * fmin((double)s->snr, 1000.0));
	}
	double d = hypot((double)s->x_mm - e->px, (double)s->y_mm - e->py);

	return (d <= m->cfg.gate_mm) ? d : HUGE_VAL;
}

/* Cost of emitter e taking spot j (NONE: no spot). */
static double cost_of(const at_match *m, const emitter *e, const at_mframe *f, int j)
{
	return (j == NONE) ? m->cfg.gate_mm : spot_cost(m, e, &f->s[j]);
}

/* Exhaustive assignment: at most MAX_EMITTERS emitters, AT_MATCH_MAX_SPOTS spots. */
static void search(const at_match *m, const emitter *em, int ne, const at_mframe *f, int i, int *cur,
		   uint32_t used, double cost, double *best, int *best_a)
{
	if (cost >= *best) {
		return;
	}
	if (i == ne) {
		*best = cost;
		memcpy(best_a, cur, (size_t)ne * sizeof(int));
		return;
	}
	for (int j = NONE; j < f->n; j++) {
		if ((j != NONE) && ((f->taken[j] != 0U) || ((used >> j) & 1U))) {
			continue;
		}
		double c = cost_of(m, &em[i], f, j);

		if (!isfinite(c)) {
			continue;
		}
		cur[i] = j;
		search(m, em, ne, f, i + 1, cur, (j == NONE) ? used : (used | (1U << j)), cost + c, best,
		       best_a);
	}
}

static uint8_t quality(float snr)
{
	return (snr >= 255.0F) ? 255U : ((snr <= 0.0F) ? 0U : (uint8_t)snr);
}

static void hit_event(at_match *m, const at_mshot *s, int w, const at_mframe *f, int j, bool ambiguous,
		      at_mevent *ev)
{
	*ev = (at_mevent){.kind = AT_EV_HIT, .node = s->node, .k = s->k, .shot_no = s->shot_no,
			  .t_break_us = s->t_break_us};
	if (f == NULL) {
		ev->result = AT_HIT_DROPPED;
	} else if (ambiguous) {
		ev->result = AT_HIT_AMBIGUOUS;
		m->stats.ambiguous++;
	} else if (j == NONE) {
		ev->result = AT_HIT_NO_SPOT;
	} else {
		double x = f->s[j].x_mm;
		double y = f->s[j].y_mm;

		at_zero_apply(&m->zero[w], &x, &y);
		ev->x_mm = (float)x;
		ev->y_mm = (float)y;
		ev->quality = quality(f->s[j].snr);
		ev->result = ((f->s[j].flags & AT_MSPOT_OFF_SCREEN) != 0U)  ? AT_HIT_OFF_SCREEN
			     : ((f->s[j].flags & AT_MSPOT_SATURATED) != 0U) ? AT_HIT_SATURATED
									     : AT_HIT_OK;
	}
	m->stats.shots++;
}

/* Settle exposure k. Returns the number of events written (at most MAX_EMITTERS). */
static int settle(at_match *m, int64_t k, at_mevent *out)
{
	at_mframe *f = frame_of(m, k);
	emitter em[MAX_EMITTERS];
	int ne = 0;
	int nev = 0;
	int64_t t_mid = (k * AT_FRAME_US) + AT_PULSE_START_US + (AT_PULSE_US / 2);

	for (int w = 0; w < m->cfg.n_weapons; w++) {
		bool shot = false;

		for (int i = 0; (i < AT_MATCH_SHOTS) && (ne < MAX_EMITTERS); i++) {
			if (m->shot[i].active && (m->shot[i].k == k) &&
			    (m->shot[i].node == m->cfg.weapon[w].node)) {
				em[ne++] = (emitter){w, EM_SHOT, i, false, 0.0, 0.0};
				shot = true;
			}
		}
		if (!shot && (slot_of(m, k) == m->cfg.weapon[w].slot) && (ne < MAX_EMITTERS)) {
			em[ne++] = (emitter){w, EM_AIM, NONE, false, 0.0, 0.0};
		}
	}
	if (f == NULL) {
		m->stats.dropped++;
		for (int i = 0; i < ne; i++) {
			if (em[i].kind == EM_SHOT) {
				hit_event(m, &m->shot[em[i].shot], em[i].weapon, NULL, NONE, false, &out[nev++]);
				m->shot[em[i].shot].active = false;
			}
		}
		return nev;
	}
	for (int i = 0; i < ne; i++) {
		em[i].has_pred = at_track_predict(&m->track[em[i].weapon], &m->cfg.track, t_mid, &em[i].px,
						  &em[i].py);
	}

	int cur[MAX_EMITTERS], best_a[MAX_EMITTERS];
	double best = HUGE_VAL;
	bool amb[MAX_EMITTERS] = {false};

	for (int i = 0; i < ne; i++) {
		best_a[i] = NONE;
	}
	search(m, em, ne, f, 0, cur, 0U, 0.0, &best, best_a);

	/* Ambiguity: exchanging two emitters' assignments costs less than ambiguity_mm more. */
	for (int a = 0; a < ne; a++) {
		for (int b = a + 1; b < ne; b++) {
			if ((best_a[a] == best_a[b]) || ((best_a[a] == NONE) && (best_a[b] == NONE))) {
				continue;
			}
			double now = cost_of(m, &em[a], f, best_a[a]) + cost_of(m, &em[b], f, best_a[b]);
			double swapped = cost_of(m, &em[a], f, best_a[b]) + cost_of(m, &em[b], f, best_a[a]);

			if (isfinite(swapped) && ((swapped - now) < m->cfg.ambiguity_mm)) {
				amb[a] = true;
				amb[b] = true;
			}
		}
	}

	for (int i = 0; i < ne; i++) {
		int j = best_a[i];
		int w = em[i].weapon;

		if (em[i].kind == EM_SHOT) {
			hit_event(m, &m->shot[em[i].shot], w, f, j, amb[i], &out[nev++]);
			m->shot[em[i].shot].active = false;
		} else if ((j != NONE) && !amb[i]) {
			double x = f->s[j].x_mm;
			double y = f->s[j].y_mm;

			at_zero_apply(&m->zero[w], &x, &y);
			out[nev++] = (at_mevent){.kind = AT_EV_AIM, .node = m->cfg.weapon[w].node, .k = k,
						 .x_mm = (float)x, .y_mm = (float)y,
						 .quality = quality(f->s[j].snr)};
		}
		if ((j != NONE) && !amb[i]) {
			at_track_update(&m->track[w], &m->cfg.track, t_mid, f->s[j].x_mm, f->s[j].y_mm);
		}
		if (j != NONE) {
			f->taken[j] = 1U;
		}
	}
	for (int j = 0; j < f->n; j++) {
		if (f->taken[j] == 0U) {
			m->stats.stray_spots++;
		}
	}
	f->settled = true;
	return nev;
}

/* A SHOT_EVENT for an exposure that is already settled: match it alone against the untaken spots. */
static int late_shot(at_match *m, int i, at_mevent *out)
{
	at_mshot *s = &m->shot[i];
	int w = weapon_index(m, s->node);
	at_mframe *f = frame_of(m, s->k);
	int j = NONE;

	m->stats.late_shots++;
	if ((f != NULL) && f->settled) {
		emitter e = {w, EM_SHOT, i, false, 0.0, 0.0};
		int64_t t_mid = (s->k * AT_FRAME_US) + AT_PULSE_START_US + (AT_PULSE_US / 2);
		double best = m->cfg.gate_mm;

		e.has_pred = at_track_predict(&m->track[w], &m->cfg.track, t_mid, &e.px, &e.py);
		for (int c = 0; c < f->n; c++) {
			double cost = (f->taken[c] != 0U) ? HUGE_VAL : spot_cost(m, &e, &f->s[c]);

			if (cost < best) {
				best = cost;
				j = c;
			}
		}
		if (j != NONE) {
			f->taken[j] = 1U;
			m->stats.stray_spots--;
		}
	} else {
		f = NULL;
	}
	hit_event(m, s, w, f, j, false, out);
	s->active = false;
	return 1;
}

int at_match_poll(at_match *m, int64_t now_us, at_mevent *out, int max)
{
	int n = 0;

	if (!m->started) {
		return 0;
	}
	/* After a long gap (no frames), skip ahead; the skipped frames are dropped. */
	int64_t due = ceil_div(now_us - m->cfg.settle_us - AT_EXPOSURE_START_US - AT_EXPOSURE_US + 1,
			       AT_FRAME_US); /* first exposure not yet due */
	if ((due - m->next_settle) > AT_MATCH_RING) {
		m->stats.dropped += (uint32_t)(due - AT_MATCH_RING - m->next_settle);
		m->next_settle = due - AT_MATCH_RING;
	}
	for (int i = 0; (i < AT_MATCH_SHOTS) && (n < max); i++) {
		if (m->shot[i].active && (m->shot[i].k < m->next_settle)) {
			n += late_shot(m, i, &out[n]);
		}
	}
	while (((at_match_exposure_end(m->next_settle) + m->cfg.settle_us) <= now_us) &&
	       ((max - n) >= MAX_EMITTERS)) {
		/* Not processed yet, and nothing newer has arrived: wait for it, up to drop_wait_us. */
		if ((frame_of(m, m->next_settle) == NULL) && (m->newest <= m->next_settle) &&
		    (now_us < (at_match_exposure_end(m->next_settle) + m->cfg.settle_us + m->cfg.drop_wait_us))) {
			break;
		}
		n += settle(m, m->next_settle, &out[n]);
		m->next_settle++;
	}
	return n;
}
