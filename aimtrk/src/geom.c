/*
 * Image <-> screen geometry, SPEC §6.3.
 */
#include <math.h>
#include <string.h>

#include "aimtrk/geom.h"
#include "aimtrk/linalg.h"

#define NEWTON_MAX 20
#define NEWTON_TOL 1e-12

void at_lens_distort(const at_lens *l, double xn, double yn, double *xd, double *yd)
{
	double r2 = (xn * xn) + (yn * yn);
	double radial = 1.0 + (l->k1 * r2) + (l->k2 * r2 * r2);

	*xd = (xn * radial) + (2.0 * l->p1 * xn * yn) + (l->p2 * (r2 + (2.0 * xn * xn)));
	*yd = (yn * radial) + (l->p1 * (r2 + (2.0 * yn * yn))) + (2.0 * l->p2 * xn * yn);
}

bool at_lens_undistort(const at_lens *l, double xd, double yd, double *xn, double *yn)
{
	double x = xd;
	double y = yd;

	for (int i = 0; i < NEWTON_MAX; i++) {
		double fx, fy;

		at_lens_distort(l, x, y, &fx, &fy);
		fx -= xd;
		fy -= yd;
		/* Jacobian of the distortion (symmetric). */
		double r2 = (x * x) + (y * y);
		double radial = 1.0 + (l->k1 * r2) + (l->k2 * r2 * r2);
		double dr = l->k1 + (2.0 * l->k2 * r2); /* d(radial) / d(r2) */
		double jxx = radial + (2.0 * x * x * dr) + (2.0 * l->p1 * y) + (6.0 * l->p2 * x);
		double jxy = (2.0 * x * y * dr) + (2.0 * l->p1 * x) + (2.0 * l->p2 * y);
		double jyy = radial + (2.0 * y * y * dr) + (6.0 * l->p1 * y) + (2.0 * l->p2 * x);
		double det = (jxx * jyy) - (jxy * jxy);

		if ((fabs(fx) < NEWTON_TOL) && (fabs(fy) < NEWTON_TOL)) {
			/*
			 * Only a root where the model is still one-to-one (Jacobian
			 * positive definite) is the physical inverse; a root in the
			 * folded part of the polynomial is not.
			 */
			if ((jxx <= 0.0) || (det <= 0.0)) {
				return false;
			}
			*xn = x;
			*yn = y;
			return true;
		}
		if (fabs(det) < 1e-12) {
			return false;
		}
		x -= ((jyy * fx) - (jxy * fy)) / det;
		y -= ((jxx * fy) - (jxy * fx)) / det;
	}
	return false;
}

bool at_h33_apply(const at_h33 *h, double x, double y, double *ox, double *oy)
{
	const double *m = h->m;
	double w = (m[6] * x) + (m[7] * y) + m[8];

	if (fabs(w) < 1e-15) {
		return false;
	}
	*ox = ((m[0] * x) + (m[1] * y) + m[2]) / w;
	*oy = ((m[3] * x) + (m[4] * y) + m[5]) / w;
	return true;
}

static void normalise_scale(at_h33 *h)
{
	double norm = 0.0;

	for (int i = 0; i < 9; i++) {
		norm += h->m[i] * h->m[i];
	}
	norm = sqrt(norm);
	double s = (fabs(h->m[8]) > (1e-12 * norm)) ? h->m[8] : norm;

	for (int i = 0; i < 9; i++) {
		h->m[i] /= s;
	}
}

bool at_h33_invert(const at_h33 *h, at_h33 *inv)
{
	const double *a = h->m;
	double c[9] = {
		(a[4] * a[8]) - (a[5] * a[7]), (a[2] * a[7]) - (a[1] * a[8]), (a[1] * a[5]) - (a[2] * a[4]),
		(a[5] * a[6]) - (a[3] * a[8]), (a[0] * a[8]) - (a[2] * a[6]), (a[2] * a[3]) - (a[0] * a[5]),
		(a[3] * a[7]) - (a[4] * a[6]), (a[1] * a[6]) - (a[0] * a[7]), (a[0] * a[4]) - (a[1] * a[3]),
	};
	double det = (a[0] * c[0]) + (a[1] * c[3]) + (a[2] * c[6]);
	double norm = 0.0;

	for (int i = 0; i < 9; i++) {
		norm = fmax(norm, fabs(a[i]));
	}
	if ((norm == 0.0) || (fabs(det) < (1e-12 * norm * norm * norm))) {
		return false;
	}
	for (int i = 0; i < 9; i++) {
		inv->m[i] = c[i] / det;
	}
	normalise_scale(inv);
	return true;
}

/* Similarity that moves the points' centroid to 0 and their mean distance to sqrt(2). */
static bool hartley(const double *p, int n, at_h33 *t)
{
	double mx = 0.0, my = 0.0, d = 0.0;

	for (int i = 0; i < n; i++) {
		mx += p[2 * i];
		my += p[(2 * i) + 1];
	}
	mx /= n;
	my /= n;
	for (int i = 0; i < n; i++) {
		d += hypot(p[2 * i] - mx, p[(2 * i) + 1] - my);
	}
	d /= n;
	if (d <= 0.0) {
		return false;
	}
	double s = sqrt(2.0) / d;

	*t = (at_h33){{s, 0.0, -s * mx, 0.0, s, -s * my, 0.0, 0.0, 1.0}};
	return true;
}

static void mul(const at_h33 *a, const at_h33 *b, at_h33 *out)
{
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			double s = 0.0;

			for (int k = 0; k < 3; k++) {
				s += a->m[(r * 3) + k] * b->m[(k * 3) + c];
			}
			out->m[(r * 3) + c] = s;
		}
	}
}

int at_h33_dlt(const double *src, const double *dst, int n, at_h33 *h)
{
	at_h33 ts, td, td_inv, hn, tmp;
	double ata[81] = {0};
	double evals[9], evecs[81];

	if ((n < 4) || !hartley(src, n, &ts) || !hartley(dst, n, &td)) {
		return -1;
	}
	for (int i = 0; i < n; i++) {
		double x, y, xp, yp;

		(void)at_h33_apply(&ts, src[2 * i], src[(2 * i) + 1], &x, &y);
		(void)at_h33_apply(&td, dst[2 * i], dst[(2 * i) + 1], &xp, &yp);
		double r1[9] = {-x, -y, -1.0, 0.0, 0.0, 0.0, xp * x, xp * y, xp};
		double r2[9] = {0.0, 0.0, 0.0, -x, -y, -1.0, yp * x, yp * y, yp};

		for (int j = 0; j < 9; j++) {
			for (int k = 0; k < 9; k++) {
				ata[(j * 9) + k] += (r1[j] * r1[k]) + (r2[j] * r2[k]);
			}
		}
	}
	if (!at_sym_eig(ata, 9, evals, evecs)) {
		return -1;
	}
	/* A unique solution needs a clear gap to the second smallest singular value. */
	if (evals[1] <= (1e-10 * evals[8])) {
		return -1;
	}
	for (int i = 0; i < 9; i++) {
		hn.m[i] = evecs[(i * 9) + 0];
	}
	if (!at_h33_invert(&td, &td_inv)) {
		return -1;
	}
	mul(&td_inv, &hn, &tmp);
	mul(&tmp, &ts, h);
	normalise_scale(h);
	return 0;
}

int at_geom_init(at_geom *g, const at_lens *l, const at_h33 *to_cam, double screen_w, double screen_h)
{
	memset(g, 0, sizeof(*g));
	g->lens = *l;
	g->to_cam = *to_cam;
	g->half_w = 0.5 * screen_w;
	g->half_h = 0.5 * screen_h;
	if ((l->f <= 0.0) || !at_h33_invert(to_cam, &g->to_screen)) {
		return -1;
	}
	return 0;
}

int at_geom_to_screen(const at_geom *g, double u, double v, double *x_mm, double *y_mm)
{
	double xn, yn;

	if (!at_lens_undistort(&g->lens, (u - g->lens.cx) / g->lens.f, (v - g->lens.cy) / g->lens.f, &xn,
			       &yn) ||
	    !at_h33_apply(&g->to_screen, xn, yn, x_mm, y_mm)) {
		return AT_GEOM_FAIL;
	}
	return ((fabs(*x_mm) <= g->half_w) && (fabs(*y_mm) <= g->half_h)) ? AT_GEOM_ON_SCREEN
									  : AT_GEOM_OFF_SCREEN;
}

bool at_geom_to_image(const at_geom *g, double x_mm, double y_mm, double *u, double *v)
{
	double xn, yn, xd, yd;

	if (!at_h33_apply(&g->to_cam, x_mm, y_mm, &xn, &yn)) {
		return false;
	}
	at_lens_distort(&g->lens, xn, yn, &xd, &yd);
	*u = (g->lens.f * xd) + g->lens.cx;
	*v = (g->lens.f * yd) + g->lens.cy;
	return true;
}
