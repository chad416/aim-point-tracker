/*
 * Screen calibration from a projected dot grid, SPEC §6.4.
 */
#include <math.h>
#include <string.h>

#include "aimtrk/calib.h"
#include "aimtrk/linalg.h"

#define NPAR       12 /* homography (8, m[8] = 1) + k1, k2, p1, p2 */
#define MIN_FIT    8
#define LM_LAMBDA0 1e-3

void at_calib_cfg_default(at_calib_cfg *cfg)
{
	*cfg = (at_calib_cfg){
		.max_fit_rms_px = 0.3,
		.max_holdout_rms_px = 0.5,
		.gate_px = 30.0,
		.min_found = 0,
		.holdout_every = 5,
		.max_iter = 100,
	};
}

/* Index of the point with the smallest (sign = -1) or largest (+1) a*x + b*y. */
static int extreme(const double *p, int n, double a, double b, double sign)
{
	int best = 0;

	for (int i = 1; i < n; i++) {
		double v = (a * p[2 * i]) + (b * p[(2 * i) + 1]);
		double w = (a * p[2 * best]) + (b * p[(2 * best) + 1]);

		if ((sign * v) > (sign * w)) {
			best = i;
		}
	}
	return best;
}

/* Predict the grid in pixels through h (screen -> normalised camera), assign the nearest dots. */
static int assign(const double *img, int n_img, const double *grid, int n_grid, const at_lens *lens,
		  const at_h33 *h, double gate_px, int *match)
{
	double dist[AT_CAL_MAX_POINTS];
	int found = 0;

	for (int i = 0; i < n_grid; i++) {
		double xn, yn, xd, yd;

		match[i] = -1;
		if (!at_h33_apply(h, grid[2 * i], grid[(2 * i) + 1], &xn, &yn)) {
			continue;
		}
		at_lens_distort(lens, xn, yn, &xd, &yd);
		double u = (lens->f * xd) + lens->cx;
		double v = (lens->f * yd) + lens->cy;

		for (int j = 0; j < n_img; j++) {
			double d = hypot(img[2 * j] - u, img[(2 * j) + 1] - v);

			if ((d <= gate_px) && ((match[i] < 0) || (d < dist[i]))) {
				match[i] = j;
				dist[i] = d;
			}
		}
	}
	/* A dot can belong to one grid point only: keep the closer claim. */
	for (int i = 0; i < n_grid; i++) {
		for (int k = i + 1; (match[i] >= 0) && (k < n_grid); k++) {
			if (match[k] == match[i]) {
				if (dist[k] < dist[i]) {
					match[i] = -1;
				} else {
					match[k] = -1;
				}
			}
		}
	}
	for (int i = 0; i < n_grid; i++) {
		found += (match[i] >= 0) ? 1 : 0;
	}
	return found;
}

/* Homography screen mm -> normalised camera through the matched points (distortion ignored). */
static int fit_h(const double *img, const double *grid, const int *match, int n_grid,
		 const at_lens *lens, at_h33 *h)
{
	double src[2 * AT_CAL_MAX_POINTS], dst[2 * AT_CAL_MAX_POINTS];
	int n = 0;

	for (int i = 0; i < n_grid; i++) {
		if (match[i] < 0) {
			continue;
		}
		src[2 * n] = grid[2 * i];
		src[(2 * n) + 1] = grid[(2 * i) + 1];
		dst[2 * n] = (img[2 * match[i]] - lens->cx) / lens->f;
		dst[(2 * n) + 1] = (img[(2 * match[i]) + 1] - lens->cy) / lens->f;
		n++;
	}
	return at_h33_dlt(src, dst, n, h);
}

int at_calib_order(const double *img, int n_img, const double *grid, int n_grid, const at_lens *lens,
		   double gate_px, int *match)
{
	if ((n_img < 4) || (n_grid < 4) || (n_grid > AT_CAL_MAX_POINTS)) {
		return AT_CAL_ORDER;
	}
	/* Corners: image y is down, screen y is up. Order: top-left, top-right, bottom-right, bottom-left. */
	int ic[4] = {extreme(img, n_img, 1.0, 1.0, -1.0), extreme(img, n_img, 1.0, -1.0, 1.0),
		     extreme(img, n_img, 1.0, 1.0, 1.0), extreme(img, n_img, 1.0, -1.0, -1.0)};
	int gc[4] = {extreme(grid, n_grid, 1.0, -1.0, -1.0), extreme(grid, n_grid, 1.0, 1.0, 1.0),
		     extreme(grid, n_grid, 1.0, -1.0, 1.0), extreme(grid, n_grid, 1.0, 1.0, -1.0)};
	int m[AT_CAL_MAX_POINTS];
	at_h33 h;

	for (int i = 0; i < 4; i++) {
		for (int j = i + 1; j < 4; j++) {
			if ((ic[i] == ic[j]) || (gc[i] == gc[j])) {
				return AT_CAL_ORDER;
			}
		}
	}
	for (int i = 0; i < n_grid; i++) {
		m[i] = -1;
	}
	for (int i = 0; i < 4; i++) {
		m[gc[i]] = ic[i];
	}
	if (fit_h(img, grid, m, n_grid, lens, &h) != 0) {
		return AT_CAL_ORDER;
	}
	(void)assign(img, n_img, grid, n_grid, lens, &h, gate_px, m);
	if (fit_h(img, grid, m, n_grid, lens, &h) != 0) {
		return AT_CAL_ORDER;
	}
	int found = assign(img, n_img, grid, n_grid, lens, &h, gate_px, m);

	memcpy(match, m, (size_t)n_grid * sizeof(int));
	return found;
}

/* Model: normalised screen point -> pixel, for parameters p (see NPAR). */
static void model(const double *p, const at_lens *nominal, double x, double y, double *u, double *v)
{
	at_lens l = *nominal;
	double w = (p[6] * x) + (p[7] * y) + 1.0;
	double xn = ((p[0] * x) + (p[1] * y) + p[2]) / w;
	double yn = ((p[3] * x) + (p[4] * y) + p[5]) / w;
	double xd, yd;

	l.k1 = p[8];
	l.k2 = p[9];
	l.p1 = p[10];
	l.p2 = p[11];
	at_lens_distort(&l, xn, yn, &xd, &yd);
	*u = (l.f * xd) + l.cx;
	*v = (l.f * yd) + l.cy;
}

static bool held_out(const at_calib_cfg *cfg, int i)
{
	return (cfg->holdout_every > 0) && ((i % cfg->holdout_every) == (cfg->holdout_every - 1));
}

/* Sum of squared residuals over the fitted points; optionally the normal equations J'J, J'r. */
static double cost(const double *p, const double *img, const double *gn, const int *match, int n_grid,
		   const at_lens *nominal, const at_calib_cfg *cfg, double *jtj, double *jtr)
{
	double c = 0.0;

	if (jtj != NULL) {
		memset(jtj, 0, NPAR * NPAR * sizeof(double));
		memset(jtr, 0, NPAR * sizeof(double));
	}
	for (int i = 0; i < n_grid; i++) {
		if ((match[i] < 0) || held_out(cfg, i)) {
			continue;
		}
		double u, v;

		model(p, nominal, gn[2 * i], gn[(2 * i) + 1], &u, &v);
		double r[2] = {u - img[2 * match[i]], v - img[(2 * match[i]) + 1]};

		c += (r[0] * r[0]) + (r[1] * r[1]);
		if (jtj == NULL) {
			continue;
		}
		double ju[NPAR], jv[NPAR];

		for (int k = 0; k < NPAR; k++) {
			double q[NPAR];
			double h = 1e-7 * fmax(1.0, fabs(p[k]));
			double u1, v1, u0, v0;

			memcpy(q, p, sizeof(q));
			q[k] = p[k] + h;
			model(q, nominal, gn[2 * i], gn[(2 * i) + 1], &u1, &v1);
			q[k] = p[k] - h;
			model(q, nominal, gn[2 * i], gn[(2 * i) + 1], &u0, &v0);
			ju[k] = (u1 - u0) / (2.0 * h);
			jv[k] = (v1 - v0) / (2.0 * h);
		}
		for (int a = 0; a < NPAR; a++) {
			for (int b = 0; b < NPAR; b++) {
				jtj[(a * NPAR) + b] += (ju[a] * ju[b]) + (jv[a] * jv[b]);
			}
			jtr[a] += (ju[a] * r[0]) + (jv[a] * r[1]);
		}
	}
	return c;
}

int at_calib_fit(const double *img, const double *grid, const int *match, int n_grid,
		 const at_lens *nominal, const at_calib_cfg *cfg, double screen_w, double screen_h,
		 at_geom *out, at_calib_report *rep)
{
	double gn[2 * AT_CAL_MAX_POINTS];
	double p[NPAR], jtj[NPAR * NPAR], jtr[NPAR];
	at_h33 ts, h0;
	int fit_match[AT_CAL_MAX_POINTS];
	int n_fit = 0;

	memset(rep, 0, sizeof(*rep));
	if ((n_grid < 4) || (n_grid > AT_CAL_MAX_POINTS)) {
		rep->status = AT_CAL_TOO_FEW;
		return rep->status;
	}
	for (int i = 0; i < n_grid; i++) {
		fit_match[i] = (held_out(cfg, i) ? -1 : match[i]);
		if (match[i] >= 0) {
			rep->n_found++;
			if (held_out(cfg, i)) {
				rep->n_holdout++;
			} else {
				n_fit++;
			}
		}
	}
	rep->n_fit = n_fit;
	int need = (cfg->min_found > 0) ? cfg->min_found : n_grid;

	if ((rep->n_found < need) || (n_fit < MIN_FIT)) {
		rep->status = AT_CAL_TOO_FEW;
		return rep->status;
	}

	/* Normalise the screen points (Hartley) so the parameters are of similar size. */
	{
		double mx = 0.0, my = 0.0, d = 0.0;

		for (int i = 0; i < n_grid; i++) {
			mx += grid[2 * i];
			my += grid[(2 * i) + 1];
		}
		mx /= n_grid;
		my /= n_grid;
		for (int i = 0; i < n_grid; i++) {
			d += hypot(grid[2 * i] - mx, grid[(2 * i) + 1] - my);
		}
		double s = sqrt(2.0) * n_grid / d;

		ts = (at_h33){{s, 0.0, -s * mx, 0.0, s, -s * my, 0.0, 0.0, 1.0}};
		for (int i = 0; i < n_grid; i++) {
			(void)at_h33_apply(&ts, grid[2 * i], grid[(2 * i) + 1], &gn[2 * i], &gn[(2 * i) + 1]);
		}
	}

	/* Start: homography through the fitted points, nominal distortion. */
	if (fit_h(img, gn, fit_match, n_grid, nominal, &h0) != 0) {
		rep->status = AT_CAL_NO_CONVERGE;
		return rep->status;
	}
	for (int k = 0; k < 8; k++) {
		p[k] = h0.m[k] / h0.m[8];
	}
	p[8] = nominal->k1;
	p[9] = nominal->k2;
	p[10] = nominal->p1;
	p[11] = nominal->p2;

	/* Levenberg-Marquardt (Marquardt's scaling of the diagonal). */
	double lambda = LM_LAMBDA0;
	double c = cost(p, img, gn, match, n_grid, nominal, cfg, jtj, jtr);
	bool converged = false;
	int it;

	for (it = 0; (it < cfg->max_iter) && !converged; it++) {
		bool stepped = false;

		while (!stepped && (lambda < 1e12)) {
			double a[NPAR * NPAR], step[NPAR], q[NPAR];

			memcpy(a, jtj, sizeof(a));
			for (int k = 0; k < NPAR; k++) {
				a[(k * NPAR) + k] += lambda * fmax(jtj[(k * NPAR) + k], 1e-12);
				step[k] = -jtr[k];
			}
			if (!at_solve(a, step, NPAR)) {
				lambda *= 10.0;
				continue;
			}
			double size = 0.0;

			for (int k = 0; k < NPAR; k++) {
				q[k] = p[k] + step[k];
				size = fmax(size, fabs(step[k]) / fmax(1e-3, fabs(p[k])));
			}
			double cq = cost(q, img, gn, match, n_grid, nominal, cfg, NULL, NULL);

			if (cq < c) {
				converged = ((c - cq) <= (1e-12 * c)) || (size < 1e-10);
				memcpy(p, q, sizeof(p));
				c = cost(p, img, gn, match, n_grid, nominal, cfg, jtj, jtr);
				lambda = fmax(lambda / 10.0, 1e-12);
				stepped = true;
			} else {
				lambda *= 10.0;
			}
		}
		if (!stepped) {
			converged = true; /* no step lowers the cost: at the minimum to working precision */
		}
	}
	rep->iterations = it;
	if (!converged) {
		rep->status = AT_CAL_NO_CONVERGE;
		return rep->status;
	}

	/* Result: to_cam = G_n * T_s. */
	at_h33 gnorm = {{p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], 1.0}};
	at_h33 to_cam;
	at_lens lens = *nominal;
	at_geom g;

	for (int r = 0; r < 3; r++) {
		for (int col = 0; col < 3; col++) {
			double s = 0.0;

			for (int k = 0; k < 3; k++) {
				s += gnorm.m[(r * 3) + k] * ts.m[(k * 3) + col];
			}
			to_cam.m[(r * 3) + col] = s;
		}
	}
	lens.k1 = p[8];
	lens.k2 = p[9];
	lens.p1 = p[10];
	lens.p2 = p[11];
	if (at_geom_init(&g, &lens, &to_cam, screen_w, screen_h) != 0) {
		rep->status = AT_CAL_NO_CONVERGE;
		return rep->status;
	}

	/* Residuals of the final geometry, fitted and held-out points. */
	double sf = 0.0, sh = 0.0;

	for (int i = 0; i < n_grid; i++) {
		double u, v;

		if ((match[i] < 0) || !at_geom_to_image(&g, grid[2 * i], grid[(2 * i) + 1], &u, &v)) {
			continue;
		}
		double e = hypot(u - img[2 * match[i]], v - img[(2 * match[i]) + 1]);

		rep->max_px = fmax(rep->max_px, e);
		if (held_out(cfg, i)) {
			sh += e * e;
		} else {
			sf += e * e;
		}
	}
	rep->fit_rms_px = sqrt(sf / n_fit);
	rep->holdout_rms_px = (rep->n_holdout > 0) ? sqrt(sh / rep->n_holdout) : 0.0;
	if ((rep->fit_rms_px > cfg->max_fit_rms_px) || (rep->holdout_rms_px > cfg->max_holdout_rms_px)) {
		rep->status = AT_CAL_RESIDUAL;
		return rep->status;
	}
	*out = g;
	rep->status = AT_CAL_OK;
	return AT_CAL_OK;
}

int at_calib_run(const double *img, int n_img, const double *grid, int n_grid, const at_lens *nominal,
		 const at_calib_cfg *cfg, double screen_w, double screen_h, at_geom *out,
		 at_calib_report *rep)
{
	int match[AT_CAL_MAX_POINTS];
	int found = at_calib_order(img, n_img, grid, n_grid, nominal, cfg->gate_px, match);

	if (found < 0) {
		memset(rep, 0, sizeof(*rep));
		rep->status = AT_CAL_ORDER;
		return rep->status;
	}
	return at_calib_fit(img, grid, match, n_grid, nominal, cfg, screen_w, screen_h, out, rep);
}
