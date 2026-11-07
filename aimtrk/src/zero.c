/*
 * Zeroing per weapon, SPEC F13.
 */
#include <math.h>
#include <string.h>

#include "aimtrk/zero.h"

void at_zero_reset(at_zero *z)
{
	memset(z, 0, sizeof(*z));
}

void at_zero_begin(at_zero *z)
{
	z->n = 0;
	z->sx = 0.0;
	z->sy = 0.0;
	z->sxx = 0.0;
	z->syy = 0.0;
}

void at_zero_add(at_zero *z, double hit_x, double hit_y, double target_x, double target_y)
{
	double dx = hit_x - target_x;
	double dy = hit_y - target_y;

	z->n++;
	z->sx += dx;
	z->sy += dy;
	z->sxx += dx * dx;
	z->syy += dy * dy;
}

int at_zero_finish(at_zero *z, int min_shots, double max_spread_mm)
{
	if ((z->n < min_shots) || (z->n < 1)) {
		return AT_ZERO_TOO_FEW;
	}
	double mx = z->sx / z->n;
	double my = z->sy / z->n;
	double var = ((z->sxx / z->n) - (mx * mx)) + ((z->syy / z->n) - (my * my));
	double spread = sqrt(fmax(var, 0.0));

	if (spread > max_spread_mm) {
		return AT_ZERO_SCATTERED;
	}
	z->valid = true;
	z->off_x = mx;
	z->off_y = my;
	z->spread_mm = spread;
	z->shots = z->n;
	return AT_ZERO_OK;
}

void at_zero_apply(const at_zero *z, double *x_mm, double *y_mm)
{
	if (z->valid) {
		*x_mm -= z->off_x;
		*y_mm -= z->off_y;
	}
}
