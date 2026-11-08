/*
 * Aim track per weapon (SPEC F07, §6.6): an alpha-beta (constant velocity)
 * filter on screen coordinates, updated with that weapon's spots. It predicts
 * the aim point at any lane time, for gating and spot assignment. A track not
 * updated for lost_us is lost; the next spot starts it again.
 */
#ifndef AIMTRK_TRACK_H_
#define AIMTRK_TRACK_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	double alpha;   /* position gain */
	double beta;    /* velocity gain */
	int64_t lost_us;
} at_track_cfg;

typedef struct {
	bool active;
	int64_t t_us;   /* lane time of the last update */
	double x, y;    /* mm */
	double vx, vy;  /* mm/s */
	uint32_t updates;
	uint32_t restarts; /* lost, then started again by a new spot */
	uint32_t ignored;  /* updates not later than the last one: ignored */
} at_track;

void at_track_cfg_default(at_track_cfg *c);
void at_track_reset(at_track *t);

/* A measured aim point at lane time t_us (mm). An update not later than the last one is ignored. */
void at_track_update(at_track *t, const at_track_cfg *c, int64_t t_us, double x, double y);

/* Predicted aim point at lane time t_us. False if there is no track or it is lost. */
bool at_track_predict(const at_track *t, const at_track_cfg *c, int64_t t_us, double *x, double *y);

#endif /* AIMTRK_TRACK_H_ */
