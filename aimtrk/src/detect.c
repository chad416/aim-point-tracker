/*
 * IR spot detection, SPEC §6.1. Integer work per pixel, floating point per
 * frame (sigma) and per spot (centroid).
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "aimtrk/detect.h"

#define DROPPED   0xFFFEU
#define AREA_CAP  0xFFF0U /* blob areas while summing (blob_of), below DROPPED */
#define HIST_BINS 511 /* frame - background, -255..255 */
#define HIST_ZERO 255
#define NOISE_STEP 4  /* noise sample: every 4th pixel of every 4th row */
#define MAD_TO_SIGMA 1.4826

void at_detect_cfg_default(at_detect_cfg *cfg, uint16_t width, uint16_t height)
{
	*cfg = (at_detect_cfg){
		.width = width,
		.height = height,
		.bg_shift = 4,  /* time constant 16 frames, 160 ms at 100 fps */
		.k_sigma = 6.0F,
		.t_min = 8.0F,
		.min_area = 3,
		.max_area = 300,
		.max_extent = 24,
		.max_elong = 3.0F,
		.min_snr = 8.0F,
		.sat_level = 255,
		.max_runs = 8192,
		.max_blobs = 1024, /* a scene change can leave hundreds of spot-sized fragments (week 4) */
	};
}

static bool cfg_ok(const at_detect_cfg *cfg)
{
	return (cfg->width >= AT_BG_CELL) && (cfg->height >= AT_BG_CELL) &&
	       ((cfg->width % AT_BG_CELL) == 0) && ((cfg->height % AT_BG_CELL) == 0) &&
	       (cfg->bg_shift <= 16U) && (cfg->max_runs >= 1U) && (cfg->max_runs < DROPPED) &&
	       (cfg->max_blobs >= 1U) && (cfg->max_blobs < DROPPED) && (cfg->max_extent >= 1U) &&
	       (cfg->max_extent <= 254U) && (cfg->k_sigma >= 0.0F) && (cfg->t_min >= 0.0F);
}

static size_t align8(size_t n)
{
	return (n + 7U) & ~(size_t)7U;
}

/* Memory layout, in order: bg, thr, cellsum, freeze, runs, blob_of, blobs, mask, bgw, hist. */
static size_t layout(const at_detect_cfg *cfg, size_t off[10])
{
	size_t cells = (size_t)(cfg->width / AT_BG_CELL) * (size_t)(cfg->height / AT_BG_CELL);
	size_t win = (size_t)(cfg->max_extent + 2U) * (size_t)(cfg->max_extent + 2U);
	size_t sizes[10] = {
		cells * sizeof(uint16_t),
		cells,
		cells * sizeof(uint16_t),
		cells,
		cfg->max_runs * sizeof(at_run),
		cfg->max_runs * sizeof(uint16_t),
		cfg->max_blobs * sizeof(at_blob),
		win,
		win * sizeof(float),
		HIST_BINS * sizeof(uint32_t),
	};
	size_t total = 0;

	for (int i = 0; i < 10; i++) {
		off[i] = total;
		total += align8(sizes[i]);
	}
	return total;
}

size_t at_detect_mem_size(const at_detect_cfg *cfg)
{
	size_t off[10];

	return cfg_ok(cfg) ? (layout(cfg, off) + 8U) : 0U; /* + 8: room to align the block */
}

int at_detect_init(at_detect *d, const at_detect_cfg *cfg, void *mem, size_t mem_size)
{
	size_t off[10];

	if (!cfg_ok(cfg) || (mem == NULL) || (mem_size < at_detect_mem_size(cfg))) {
		return -1;
	}
	uint8_t *base = (uint8_t *)mem;

	base += (8U - ((uintptr_t)base & 7U)) & 7U;
	(void)layout(cfg, off);
	/* Touch all of it now: the page faults happen at start-up, not in the first frames (P05). */
	memset(mem, 0, mem_size);

	memset(d, 0, sizeof(*d));
	d->cfg = *cfg;
	d->cw = cfg->width / AT_BG_CELL;
	d->ch = cfg->height / AT_BG_CELL;
	d->bg = (uint16_t *)(void *)(base + off[0]);
	d->thr = base + off[1];
	d->cellsum = (uint16_t *)(void *)(base + off[2]);
	d->freeze = base + off[3];
	d->runs = (at_run *)(void *)(base + off[4]);
	d->blob_of = (uint16_t *)(void *)(base + off[5]);
	d->blobs = (at_blob *)(void *)(base + off[6]);
	d->mask = base + off[7];
	d->bgw = (float *)(void *)(base + off[8]);
	d->hist = (uint32_t *)(void *)(base + off[9]);
	return 0;
}

static void cell_sums(at_detect *d, const uint8_t *img, size_t stride)
{
	memset(d->cellsum, 0, (size_t)d->cw * (size_t)d->ch * sizeof(uint16_t));
	for (int y = 0; y < d->cfg.height; y++) {
		const uint8_t *row = img + ((size_t)y * stride);
		uint16_t *cs = d->cellsum + ((size_t)(y / AT_BG_CELL) * (size_t)d->cw);

		for (int x = 0; x < d->cfg.width; x++) {
			cs[x / AT_BG_CELL] += row[x];
		}
	}
}

/* A cell sum is 16 pixels, so sum * 16 is the cell mean in DN * 256 (at most 65280). */
#define SUM_TO_BG(s) ((int32_t)(s) * (256 / (AT_BG_CELL * AT_BG_CELL)))

void at_detect_learn(at_detect *d, const uint8_t *img, size_t stride)
{
	cell_sums(d, img, stride);
	for (int i = 0; i < (d->cw * d->ch); i++) {
		d->bg[i] = (uint16_t)SUM_TO_BG(d->cellsum[i]);
	}
	d->bg_valid = true;
}

void at_detect_reset(at_detect *d)
{
	d->bg_valid = false;
}

float at_detect_bg(const at_detect *d, int x, int y)
{
	return (float)d->bg[((y / AT_BG_CELL) * d->cw) + (x / AT_BG_CELL)] / 256.0F;
}

const at_detect_stats *at_detect_get_stats(const at_detect *d)
{
	return &d->stats;
}

/* Median of a histogram of n samples, bins of width 1; bin 0 starts at lo. */
static double hist_median(const uint32_t *h, int bins, double lo, uint32_t n)
{
	double target = 0.5 * (double)n;
	double cum = 0.0;

	for (int i = 0; i < bins; i++) {
		if ((h[i] > 0U) && ((cum + (double)h[i]) >= target)) {
			return lo + (double)i + ((target - cum) / (double)h[i]);
		}
		cum += (double)h[i];
	}
	return lo + (double)bins;
}

float at_detect_noise(at_detect *d, const uint8_t *img, size_t stride)
{
	uint32_t *h = d->hist;
	uint32_t n = 0;

	memset(h, 0, HIST_BINS * sizeof(uint32_t));
	for (int y = NOISE_STEP / 2; y < d->cfg.height; y += NOISE_STEP) {
		const uint8_t *row = img + ((size_t)y * stride);
		const uint16_t *bg = d->bg + ((size_t)(y / AT_BG_CELL) * (size_t)d->cw);

		for (int x = NOISE_STEP / 2; x < d->cfg.width; x += NOISE_STEP) {
			int b = ((int)bg[x / AT_BG_CELL] + 128) >> 8;

			h[(int)row[x] - b + HIST_ZERO]++;
			n++;
		}
	}
	/* Median of the difference, in bins centred on integers. */
	double med = hist_median(h, HIST_BINS, -255.5, n);
	int m = (int)floor(med + 0.5);
	/*
	 * Folded histogram of |d - m| into h[0..255] is not possible in place, so
	 * walk it directly: bin 0 covers 0..0.5 (one integer value), bin k >= 1
	 * covers k - 0.5..k + 0.5 (two values, m - k and m + k).
	 */
	double target = 0.5 * (double)n;
	double cum = 0.0;
	double mad = 0.0;

	for (int k = 0; k < HIST_BINS; k++) {
		int lo = m - k + HIST_ZERO;
		int hi = m + k + HIST_ZERO;
		uint32_t c = 0;

		if ((lo >= 0) && (lo < HIST_BINS)) {
			c += h[lo];
		}
		if ((k > 0) && (hi >= 0) && (hi < HIST_BINS)) {
			c += h[hi];
		}
		if ((c > 0U) && ((cum + (double)c) >= target)) {
			double frac = (target - cum) / (double)c;

			mad = (k == 0) ? (0.5 * frac) : (((double)k - 0.5) + frac);
			break;
		}
		cum += (double)c;
	}
	d->stats.sigma = (float)(MAD_TO_SIGMA * mad);
	return d->stats.sigma;
}

static uint16_t find(at_run *runs, uint16_t i)
{
	while (runs[i].label != i) {
		runs[i].label = runs[runs[i].label].label; /* path halving */
		i = runs[i].label;
	}
	return i;
}

static void unite(at_run *runs, uint16_t a, uint16_t b)
{
	uint16_t ra = find(runs, a);
	uint16_t rb = find(runs, b);

	if (ra < rb) {
		runs[rb].label = ra;
	} else if (rb < ra) {
		runs[ra].label = rb;
	}
}

/* Threshold, cell sums and runs in one pass over the frame. Returns the run count. */
static int scan(at_detect *d, const uint8_t *img, size_t stride)
{
	at_run *runs = d->runs;
	int n = 0;
	int prev_start = 0, prev_end = 0;

	memset(d->cellsum, 0, (size_t)d->cw * (size_t)d->ch * sizeof(uint16_t));
	for (int y = 0; y < d->cfg.height; y++) {
		const uint8_t *row = img + ((size_t)y * stride);
		const uint8_t *thr = d->thr + ((size_t)(y / AT_BG_CELL) * (size_t)d->cw);
		uint16_t *cs = d->cellsum + ((size_t)(y / AT_BG_CELL) * (size_t)d->cw);
		int row_start = n;
		int j = prev_start;
		int x = 0;

		while (x < d->cfg.width) {
			uint8_t p = row[x];

			cs[x / AT_BG_CELL] += p;
			if (p <= thr[x / AT_BG_CELL]) {
				x++;
				continue;
			}
			int x0 = x;

			for (x++; x < d->cfg.width; x++) {
				p = row[x];
				cs[x / AT_BG_CELL] += p;
				if (p <= thr[x / AT_BG_CELL]) {
					break;
				}
			}
			int x1 = x - 1;

			x++; /* row[x] is below the threshold and already in the cell sum */
			if (n >= d->cfg.max_runs) {
				d->stats.overflow |= AT_OVF_RUNS;
				continue;
			}
			runs[n] = (at_run){(uint16_t)x0, (uint16_t)x1, (uint16_t)y, (uint16_t)n};
			/* Join runs of the previous row that touch this one (8-connected). */
			while ((j < prev_end) && ((runs[j].x1 + 1) < x0)) {
				j++;
			}
			for (int k = j; (k < prev_end) && (runs[k].x0 <= (x1 + 1)); k++) {
				unite(runs, (uint16_t)k, (uint16_t)n);
			}
			n++;
		}
		prev_start = row_start;
		prev_end = n;
	}
	return n;
}

/* Paint the blob's runs, dilated by one pixel, into the window mask; fill the window background. */
static void window(at_detect *d, const at_blob *b, int nruns, at_cwin *win)
{
	int x0 = (b->x0 > 0U) ? (b->x0 - 1) : 0;
	int y0 = (b->y0 > 0U) ? (b->y0 - 1) : 0;
	int x1 = ((b->x1 + 1) < d->cfg.width) ? (b->x1 + 1) : (d->cfg.width - 1);
	int y1 = ((b->y1 + 1) < d->cfg.height) ? (b->y1 + 1) : (d->cfg.height - 1);
	int w = x1 - x0 + 1;
	int h = y1 - y0 + 1;

	memset(d->mask, 0, (size_t)w * (size_t)h);
	for (int i = 0; i < nruns; i++) {
		const at_run *r = &d->runs[i];

		if ((r->label != b->root) || (r->y < b->y0) || (r->y > b->y1)) {
			continue;
		}
		int rx0 = ((r->x0 - 1) > x0) ? (r->x0 - 1) : x0;
		int rx1 = ((r->x1 + 1) < x1) ? (r->x1 + 1) : x1;

		for (int yy = r->y - 1; yy <= (r->y + 1); yy++) {
			if ((yy < y0) || (yy > y1)) {
				continue;
			}
			memset(&d->mask[((yy - y0) * w) + (rx0 - x0)], 1, (size_t)(rx1 - rx0 + 1));
		}
	}
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			d->bgw[(r * w) + c] = at_detect_bg(d, x0 + c, y0 + r);
		}
	}
	win->x0 = x0;
	win->y0 = y0;
	win->w = w;
	win->h = h;
	win->mask = d->mask;
	win->bg = d->bgw;
	win->bg_const = 0.0F;
}

static void freeze_cells(at_detect *d, const at_blob *b)
{
	int cx0 = (b->x0 / AT_BG_CELL) - 1;
	int cx1 = (b->x1 / AT_BG_CELL) + 1;
	int cy0 = (b->y0 / AT_BG_CELL) - 1;
	int cy1 = (b->y1 / AT_BG_CELL) + 1;

	for (int cy = (cy0 < 0) ? 0 : cy0; cy <= cy1 && cy < d->ch; cy++) {
		for (int cx = (cx0 < 0) ? 0 : cx0; cx <= cx1 && cx < d->cw; cx++) {
			d->freeze[(cy * d->cw) + cx] = 1U;
		}
	}
}

#define RECOVER_R        6    /* window half-size around a peak found inside a large blob, px */
#define RECOVER_PEAKS    4    /* peaks looked at per large blob */
#define RECOVER_CONTRAST 2.0  /* a peak above its surroundings, in detection thresholds */
#define RECOVER_MAX_SIG  2.5F /* a recovered spot is spot-sized: sig_major at most this, px */

static bool near_peak(const int *px, const int *py, int n, int x, int y)
{
	for (int i = 0; i < n; i++) {
		if ((abs(x - px[i]) <= RECOVER_R) && (abs(y - py[i]) <= RECOVER_R)) {
			return true;
		}
	}
	return false;
}

/*
 * Step 6: spots inside a blob that was too large. The brightest pixels of the
 * blob (above the background) are taken in turn, and each is measured against
 * its own surroundings: the brightest pixel on the border of a
 * (2 RECOVER_R + 1)^2 window around it. A spot stands out from that by at
 * least RECOVER_CONTRAST detection thresholds and is spot-sized; noise on the
 * bright side of a scene change does not. It is located against that level.
 * Returns the new spot count.
 */
static int recover(at_detect *d, const at_blob *b, int nruns, const uint8_t *img, size_t stride, float sigma,
		   int32_t delta256, at_spot *spots, int nspots, int max_spots)
{
	const at_detect_cfg *cfg = &d->cfg;
	int px[RECOVER_PEAKS], py[RECOVER_PEAKS];
	int np = 0;
	double delta = (double)delta256 / 256.0;
	double sig_n = (sigma > 0.5F) ? (double)sigma : 0.5;
	uint8_t core[(2 * RECOVER_R + 1) * (2 * RECOVER_R + 1)];

	if (((2 * RECOVER_R) + 1) > (cfg->max_extent + 2)) {
		return nspots; /* window tables too small for this configuration */
	}
	while (np < RECOVER_PEAKS) {
		double best = 0.0;
		int bx = -1, by = -1;

		for (int i = 0; i < nruns; i++) {
			const at_run *r = &d->runs[i];

			if (r->label != b->root) {
				continue;
			}
			const uint8_t *row = img + ((size_t)r->y * stride);

			for (int x = r->x0; x <= r->x1; x++) {
				double ex = (double)row[x] - (double)at_detect_bg(d, x, r->y);

				if ((ex > best) && !near_peak(px, py, np, x, r->y)) {
					best = ex;
					bx = x;
					by = r->y;
				}
			}
		}
		if ((bx < 0) || (best <= (RECOVER_CONTRAST * delta))) {
			break; /* nothing left that could stand out by enough */
		}
		px[np] = bx;
		py[np] = by;
		np++;

		int x0 = (bx > RECOVER_R) ? (bx - RECOVER_R) : 0;
		int y0 = (by > RECOVER_R) ? (by - RECOVER_R) : 0;
		int x1 = ((bx + RECOVER_R) < cfg->width) ? (bx + RECOVER_R) : (cfg->width - 1);
		int y1 = ((by + RECOVER_R) < cfg->height) ? (by + RECOVER_R) : (cfg->height - 1);
		int w = x1 - x0 + 1;
		int hh = y1 - y0 + 1;
		double ring = -HUGE_VAL;

		for (int r = 0; r < hh; r++) {
			const uint8_t *row = img + ((size_t)(y0 + r) * stride);

			for (int c = 0; c < w; c++) {
				if ((r == 0) || (c == 0) || (r == (hh - 1)) || (c == (w - 1))) {
					double ex = (double)row[x0 + c] - (double)at_detect_bg(d, x0 + c, y0 + r);

					ring = (ex > ring) ? ex : ring;
				}
			}
		}
		double contrast = best - ring;

		if ((contrast <= (RECOVER_CONTRAST * delta)) || ((contrast / sig_n) < (double)cfg->min_snr)) {
			continue;
		}
		float level = (float)ring;
		int area = 0;

		for (int r = 0; r < hh; r++) {
			const uint8_t *row = img + ((size_t)(y0 + r) * stride);

			for (int c = 0; c < w; c++) {
				float lvl = at_detect_bg(d, x0 + c, y0 + r) + level;

				d->bgw[(r * w) + c] = lvl;
				core[(r * w) + c] = (((double)row[x0 + c] - (double)lvl) > delta) ? 1U : 0U;
				area += core[(r * w) + c];
			}
		}
		if (area < cfg->min_area) {
			continue;
		}
		/* The spot's pixels and a one-pixel border. */
		memset(d->mask, 0, (size_t)w * (size_t)hh);
		for (int r = 0; r < hh; r++) {
			for (int c = 0; c < w; c++) {
				if (core[(r * w) + c] == 0U) {
					continue;
				}
				for (int rr = (r > 0) ? (r - 1) : 0; (rr <= (r + 1)) && (rr < hh); rr++) {
					for (int cc = (c > 0) ? (c - 1) : 0; (cc <= (c + 1)) && (cc < w); cc++) {
						d->mask[(rr * w) + cc] = 1U;
					}
				}
			}
		}
		at_cwin win = {.px = img + ((size_t)y0 * stride) + (size_t)x0, .stride = stride, .x0 = x0, .y0 = y0,
			       .w = w, .h = hh, .mask = d->mask, .bg = d->bgw, .bg_const = 0.0F};
		at_spot s;

		if ((at_centroid(&win, sigma, cfg->sat_level, &s) != 0) || (s.snr < cfg->min_snr) ||
		    (s.sig_minor <= 0.0F) || ((s.sig_major / s.sig_minor) > cfg->max_elong) ||
		    (s.sig_major > RECOVER_MAX_SIG)) {
			continue;
		}
		if (nspots >= max_spots) {
			d->stats.overflow |= AT_OVF_SPOTS;
			continue;
		}
		s.area = (uint16_t)area;
		s.flags |= AT_SPOT_ON_CHANGE;
		if ((x0 == 0) || (y0 == 0) || (x1 == (cfg->width - 1)) || (y1 == (cfg->height - 1))) {
			s.flags |= AT_SPOT_EDGE;
		}
		spots[nspots++] = s;
		d->stats.recovered++;
	}
	return nspots;
}

/*
 * A spot stands out from its surroundings: the brightest pixel on the ring
 * RECOVER_R out (the border of the (2 RECOVER_R + 1)^2 window around it; where
 * the image border cuts the window, that side has no ring) is at most
 * half its peak, or the peak is at least min_snr noise sigmas above it. A
 * fragment of a scene change whose step is about one detection threshold high
 * (noise splits it into small blobs) fails: its surroundings are the step
 * itself (week 4, the demo session at an 8 DN projector leak).
 */
static bool stands_out(const at_detect *d, const uint8_t *img, size_t stride, const at_spot *s, float sigma)
{
	const at_detect_cfg *cfg = &d->cfg;
	int cx = (int)floor((double)s->x + 0.5);
	int cy = (int)floor((double)s->y + 0.5);
	int x0 = (cx > RECOVER_R) ? (cx - RECOVER_R) : 0;
	int y0 = (cy > RECOVER_R) ? (cy - RECOVER_R) : 0;
	int x1 = ((cx + RECOVER_R) < cfg->width) ? (cx + RECOVER_R) : (cfg->width - 1);
	int y1 = ((cy + RECOVER_R) < cfg->height) ? (cy + RECOVER_R) : (cfg->height - 1);
	double ring = 0.0; /* no ring at all (a window cut on every side): nothing to compare with */

	for (int y = y0; y <= y1; y++) {
		const uint8_t *row = img + ((size_t)y * stride);

		for (int x = x0; x <= x1; x++) {
			if ((abs(y - cy) == RECOVER_R) || (abs(x - cx) == RECOVER_R)) {
				double ex = (double)row[x] - (double)at_detect_bg(d, x, y);

				ring = (ex > ring) ? ex : ring;
			}
		}
	}
	double peak = (double)s->peak;
	double sig_n = (sigma > 0.5F) ? (double)sigma : 0.5;

	return (ring <= (0.5 * peak)) || ((peak - ring) >= ((double)cfg->min_snr * sig_n));
}

int at_detect_frame(at_detect *d, const uint8_t *img, size_t stride, at_spot *spots, int max_spots)
{
	const at_detect_cfg *cfg = &d->cfg;
	int ncells = d->cw * d->ch;
	int nspots = 0;

	memset(&d->stats, 0, sizeof(d->stats));
	if (!d->bg_valid) {
		at_detect_learn(d, img, stride);
		d->stats.learning = true;
		return 0;
	}

	/*
	 * 2-3: noise, then the threshold per cell, over the highest background of
	 * the cell and its 8 neighbours. A cell across a sharp edge (a window
	 * frame, a person) averages its bright and dark parts; against its own
	 * mean the bright part would sit near the threshold and its noise would
	 * make spots (week 3, IT06). On a flat background this adds about 0.3 DN.
	 */
	float sigma = at_detect_noise(d, img, stride);
	double delta = fmax((double)cfg->k_sigma * (double)sigma, (double)cfg->t_min);
	int32_t delta256 = (int32_t)floor((delta * 256.0) + 0.5);

	/* Separable 3 x 3 maximum: along rows into cellsum (free until the scan), then along columns. */
	uint16_t *hmax = d->cellsum;

	for (int cy = 0; cy < d->ch; cy++) {
		const uint16_t *b = &d->bg[cy * d->cw];
		uint16_t *o = &hmax[cy * d->cw];

		for (int cx = 0; cx < d->cw; cx++) {
			uint16_t v = b[cx];

			if ((cx > 0) && (b[cx - 1] > v)) {
				v = b[cx - 1];
			}
			if (((cx + 1) < d->cw) && (b[cx + 1] > v)) {
				v = b[cx + 1];
			}
			o[cx] = v;
		}
	}
	for (int cy = 0; cy < d->ch; cy++) {
		const uint16_t *up = &hmax[((cy > 0) ? (cy - 1) : cy) * d->cw];
		const uint16_t *mid = &hmax[cy * d->cw];
		const uint16_t *dn = &hmax[(((cy + 1) < d->ch) ? (cy + 1) : cy) * d->cw];

		for (int cx = 0; cx < d->cw; cx++) {
			uint16_t v = (up[cx] > mid[cx]) ? up[cx] : mid[cx];
			int32_t t = ((int32_t)((dn[cx] > v) ? dn[cx] : v) + delta256) >> 8;

			d->thr[(cy * d->cw) + cx] = (t > 255) ? 255U : (uint8_t)t;
		}
	}

	/* 4: runs, then blobs. */
	int nruns = scan(d, img, stride);
	int nblobs = 0;

	/*
	 * Areas first, summed at each root in blob_of, so that a blob too small to
	 * be a spot never takes a place in the blob table: a scene change about
	 * one threshold high breaks up into hundreds of them in the noise, and a
	 * full table dropped every blob after them, spots included (week 4, IT06
	 * at 8 DN). A blob's root is its first run (unite() keeps the smaller
	 * index), so the roots come in raster order. The roots are final now and
	 * stay in the labels, so window() can compare them directly.
	 */
	for (int i = 0; i < nruns; i++) {
		d->runs[i].label = find(d->runs, (uint16_t)i);
		d->blob_of[i] = 0U;
	}
	for (int i = 0; i < nruns; i++) {
		const at_run *r = &d->runs[i];
		uint32_t a = (uint32_t)d->blob_of[r->label] + (uint32_t)(r->x1 - r->x0 + 1);

		d->blob_of[r->label] = (uint16_t)((a < AREA_CAP) ? a : AREA_CAP);
	}
	for (int i = 0; i < nruns; i++) {
		at_run *r = &d->runs[i];

		if (r->label == (uint16_t)i) {
			if (d->blob_of[i] < cfg->min_area) {
				d->stats.rejected[AT_REJ_SMALL]++;
				d->blob_of[i] = DROPPED;
			} else if (nblobs >= cfg->max_blobs) {
				d->stats.overflow |= AT_OVF_BLOBS;
				d->blob_of[i] = DROPPED;
			} else {
				d->blobs[nblobs] = (at_blob){(uint16_t)i, r->x0, r->x1, r->y, r->y, 0U};
				d->blob_of[i] = (uint16_t)nblobs++;
			}
		}
		if (d->blob_of[r->label] == DROPPED) {
			continue;
		}
		at_blob *b = &d->blobs[d->blob_of[r->label]];

		b->area += (uint32_t)(r->x1 - r->x0 + 1);
		b->x0 = (r->x0 < b->x0) ? r->x0 : b->x0;
		b->x1 = (r->x1 > b->x1) ? r->x1 : b->x1;
		b->y1 = r->y; /* runs come in row order */
	}
	/* 5: rejection and centroids. */
	memset(d->freeze, 0, (size_t)ncells);
	for (int i = 0; i < nblobs; i++) {
		const at_blob *b = &d->blobs[i];
		int ext_w = b->x1 - b->x0 + 1;
		int ext_h = b->y1 - b->y0 + 1;

		if ((b->area > cfg->max_area) || (ext_w > cfg->max_extent) || (ext_h > cfg->max_extent)) {
			d->stats.rejected[AT_REJ_LARGE]++;
			nspots = recover(d, b, nruns, img, stride, sigma, delta256, spots, nspots, max_spots);
			continue;
		}
		at_cwin win;
		at_spot s;

		window(d, b, nruns, &win);
		win.px = img + ((size_t)win.y0 * stride) + (size_t)win.x0;
		win.stride = stride;
		if ((at_centroid(&win, sigma, cfg->sat_level, &s) != 0) || (s.snr < cfg->min_snr)) {
			d->stats.rejected[AT_REJ_WEAK]++;
			continue;
		}
		if ((s.sig_minor <= 0.0F) || ((s.sig_major / s.sig_minor) > cfg->max_elong)) {
			d->stats.rejected[AT_REJ_ELONGATED]++;
			continue;
		}
		if (!stands_out(d, img, stride, &s, sigma)) {
			d->stats.rejected[AT_REJ_WEAK]++; /* weak against its own surroundings */
			continue;
		}
		freeze_cells(d, b);
		if (nspots >= max_spots) {
			d->stats.overflow |= AT_OVF_SPOTS;
			continue;
		}
		s.area = (b->area > UINT16_MAX) ? UINT16_MAX : (uint16_t)b->area;
		if ((b->x0 == 0U) || (b->y0 == 0U) || ((b->x1 + 1) >= cfg->width) ||
		    ((b->y1 + 1) >= cfg->height)) {
			s.flags |= AT_SPOT_EDGE;
		}
		spots[nspots++] = s;
	}

	/*
	 * 1: background update, except under spots. A cell whose mean moved by more
	 * than half the detection threshold is a change of the scene (a person
	 * walking through, the projected image changing), not noise: it takes the
	 * new value at once instead of trailing it for many frames (week 3, IT06).
	 */
	int64_t bg_sum = 0;
	int32_t snap = delta256 / 2;

	for (int i = 0; i < ncells; i++) {
		if (d->freeze[i] != 0U) {
			d->stats.frozen_cells++;
		} else {
			int32_t mean = SUM_TO_BG(d->cellsum[i]);
			int32_t diff = mean - (int32_t)d->bg[i];

			if ((diff > snap) || (diff < -snap)) {
				d->bg[i] = (uint16_t)mean;
				d->stats.snapped_cells++;
			} else {
				/* Truncates toward zero; moves toward a value in 0..65280, so it stays in range. */
				d->bg[i] = (uint16_t)((int32_t)d->bg[i] + (diff / (1 << cfg->bg_shift)));
			}
		}
		bg_sum += d->bg[i];
	}
	d->stats.bg_mean = (float)((double)bg_sum / (256.0 * (double)ncells));
	d->stats.runs = (uint32_t)nruns;
	d->stats.blobs = (uint32_t)nblobs;
	d->stats.spots = (uint32_t)nspots;
	return nspots;
}
