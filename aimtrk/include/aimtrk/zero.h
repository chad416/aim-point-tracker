/*
 * Zeroing per weapon (SPEC F13).
 *
 * The shooter fires N shots at a zeroing target. The mean offset between hit
 * and target is the emitter-to-bore misalignment (plus the mean aim error of
 * those shots); it is subtracted from that weapon's later hits. A new zeroing
 * replaces the offset only when it succeeds; until then the old one stays.
 */
#ifndef AIMTRK_ZERO_H_
#define AIMTRK_ZERO_H_

#include <stdbool.h>

#define AT_ZERO_OK        0
#define AT_ZERO_TOO_FEW   (-1) /* fewer shots than min_shots */
#define AT_ZERO_SCATTERED (-2) /* group spread over max_spread_mm: not a steady group */

typedef struct {
	/* accumulating */
	int n;
	double sx, sy, sxx, syy;
	/* in use */
	bool valid;
	double off_x, off_y; /* mm, subtracted from hits */
	double spread_mm;    /* RMS radius of the group the offset came from */
	int shots;
} at_zero;

/* No offset, nothing accumulated. */
void at_zero_reset(at_zero *z);

/* Start a new zeroing; the offset in use is kept. */
void at_zero_begin(at_zero *z);

/* One zeroing shot: hit and target, screen mm. */
void at_zero_add(at_zero *z, double hit_x, double hit_y, double target_x, double target_y);

/* Finish: on AT_ZERO_OK the new offset is in use. */
int at_zero_finish(at_zero *z, int min_shots, double max_spread_mm);

/* Apply the offset in use (no change if none). */
void at_zero_apply(const at_zero *z, double *x_mm, double *y_mm);

#endif /* AIMTRK_ZERO_H_ */
