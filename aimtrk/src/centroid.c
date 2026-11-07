/*
 * Sub-pixel spot location, SPEC §6.2. Per-spot work, so double precision.
 */
#include <math.h>
#include <stdbool.h>

#include "aimtrk/centroid.h"
#include "aimtrk/linalg.h"

#define RIM_PARAMS     6
#define RIM_MIN_PIXELS 8

static inline bool use_px(const at_cwin *win, int i)
{
	return (win->mask == NULL) || (win->mask[i] != 0U);
}

static inline double bg_at(const at_cwin *win, int i)
{
	return (win->bg != NULL) ? (double)win->bg[i] : (double)win->bg_const;
}

int at_centroid_rim(const at_cwin *win, uint8_t sat_level, float min_signal, float *x, float *y)
{
	double ata[RIM_PARAMS * RIM_PARAMS] = {0};
	double atb[RIM_PARAMS] = {0};
	/* Coordinates relative to the window centre keep the normal equations well conditioned. */
	double uc = 0.5 * (double)(win->w - 1);
	double vc = 0.5 * (double)(win->h - 1);
	int n = 0;

	for (int r = 0; r < win->h; r++) {
		const uint8_t *row = win->px + ((size_t)r * win->stride);

		for (int c = 0; c < win->w; c++) {
			int i = (r * win->w) + c;

			if (!use_px(win, i) || (row[c] >= sat_level)) {
				continue;
			}
			double s = (double)row[c] - bg_at(win, i);

			if (s <= (double)min_signal) {
				continue;
			}
			double u = (double)c - uc;
			double v = (double)r - vc;
			double phi[RIM_PARAMS] = {1.0, u, v, u * u, u * v, v * v};
			double w = s * s; /* least squares on ln(s), weighted to match a fit on s */
			double ls = log(s);

			for (int j = 0; j < RIM_PARAMS; j++) {
				for (int k = 0; k < RIM_PARAMS; k++) {
					ata[(j * RIM_PARAMS) + k] += w * phi[j] * phi[k];
				}
				atb[j] += w * phi[j] * ls;
			}
			n++;
		}
	}
	if ((n < RIM_MIN_PIXELS) || !at_solve(ata, atb, RIM_PARAMS)) {
		return -1;
	}
	/* Peak where the gradient is zero: [2d e; e 2f] (u v)' = -(b c)'. */
	double b = atb[1], c = atb[2], d = atb[3], e = atb[4], f = atb[5];
	double det = (4.0 * d * f) - (e * e);

	if ((d >= 0.0) || (f >= 0.0) || (det <= 0.0)) {
		return -1; /* not a peak */
	}
	double u0 = ((-2.0 * f * b) + (c * e)) / det;
	double v0 = ((-2.0 * d * c) + (e * b)) / det;

	if ((fabs(u0) > (uc + 1.0)) || (fabs(v0) > (vc + 1.0))) {
		return -1; /* outside the window */
	}
	*x = (float)((double)win->x0 + uc + u0);
	*y = (float)((double)win->y0 + vc + v0);
	return 0;
}

int at_centroid(const at_cwin *win, float sigma, uint8_t sat_level, at_spot *spot)
{
	double s0 = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
	double peak = 0.0;
	unsigned int n_sat = 0;

	for (int r = 0; r < win->h; r++) {
		const uint8_t *row = win->px + ((size_t)r * win->stride);

		for (int c = 0; c < win->w; c++) {
			int i = (r * win->w) + c;

			if (!use_px(win, i)) {
				continue;
			}
			double s = (double)row[c] - bg_at(win, i);

			if (row[c] >= sat_level) {
				n_sat++;
			}
			if (s > peak) {
				peak = s;
			}
			if (s <= 0.0) {
				continue;
			}
			/* Moments about the window origin; shifted to the centroid below. */
			double u = (double)c;
			double v = (double)r;

			s0 += s;
			sx += s * u;
			sy += s * v;
			sxx += s * u * u;
			syy += s * v * v;
			sxy += s * u * v;
		}
	}
	if (s0 <= 0.0) {
		return -1;
	}
	double mu = sx / s0;
	double mv = sy / s0;
	double cuu = (sxx / s0) - (mu * mu);
	double cvv = (syy / s0) - (mv * mv);
	double cuv = (sxy / s0) - (mu * mv);
	double half_tr = 0.5 * (cuu + cvv);
	double disc = sqrt(fmax((half_tr * half_tr) - ((cuu * cvv) - (cuv * cuv)), 0.0));
	double sig_n = (sigma > 0.5F) ? (double)sigma : 0.5; /* floor: quantised, noise-free frames */

	spot->cx = (float)((double)win->x0 + mu);
	spot->cy = (float)((double)win->y0 + mv);
	spot->x = spot->cx;
	spot->y = spot->cy;
	spot->sum = (float)s0;
	spot->peak = (float)peak;
	spot->snr = (float)(peak / sig_n);
	spot->sig_major = (float)sqrt(fmax(half_tr + disc, 0.0));
	spot->sig_minor = (float)sqrt(fmax(half_tr - disc, 0.0));
	spot->n_sat = (n_sat > UINT16_MAX) ? UINT16_MAX : (uint16_t)n_sat;
	spot->flags = 0U;

	if (n_sat > 0U) {
		float rx, ry;

		spot->flags |= AT_SPOT_SATURATED;
		if (at_centroid_rim(win, sat_level, (float)(4.0 * sig_n), &rx, &ry) == 0) {
			spot->x = rx;
			spot->y = ry;
			spot->flags |= AT_SPOT_RIM;
		} else {
			spot->flags |= AT_SPOT_RIM_FAILED;
		}
	}
	return 0;
}
