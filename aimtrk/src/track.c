/*
 * Aim track per weapon, SPEC §6.6.
 */
#include <string.h>

#include "aimtrk/track.h"

void at_track_cfg_default(at_track_cfg *c)
{
	/* 25-50 Hz updates; the aim point moves smoothly between them. */
	*c = (at_track_cfg){.alpha = 0.7, .beta = 0.3, .lost_us = 500000};
}

void at_track_reset(at_track *t)
{
	memset(t, 0, sizeof(*t));
}

static bool lost(const at_track *t, const at_track_cfg *c, int64_t t_us)
{
	return !t->active || ((t_us - t->t_us) > c->lost_us);
}

void at_track_update(at_track *t, const at_track_cfg *c, int64_t t_us, double x, double y)
{
	if (t->active && (t_us <= t->t_us)) {
		/* Not after the last update (a second spot for the same pulse, a stale one): ignored. */
		t->ignored++;
		return;
	}
	if (lost(t, c, t_us)) {
		if (t->active) {
			t->restarts++;
		}
		/* (Re)start: position only. */
		t->active = true;
		t->t_us = t_us;
		t->x = x;
		t->y = y;
		t->vx = 0.0;
		t->vy = 0.0;
		t->updates = 1U;
		return;
	}
	double dt = (double)(t_us - t->t_us) * 1e-6;
	double px = t->x + (t->vx * dt);
	double py = t->y + (t->vy * dt);
	double rx = x - px;
	double ry = y - py;

	t->x = px + (c->alpha * rx);
	t->y = py + (c->alpha * ry);
	t->vx += (c->beta / dt) * rx;
	t->vy += (c->beta / dt) * ry;
	t->t_us = t_us;
	t->updates++;
}

bool at_track_predict(const at_track *t, const at_track_cfg *c, int64_t t_us, double *x, double *y)
{
	if (lost(t, c, t_us)) {
		return false;
	}
	double dt = (double)(t_us - t->t_us) * 1e-6;

	*x = t->x + (t->vx * dt);
	*y = t->y + (t->vy * dt);
	return true;
}
