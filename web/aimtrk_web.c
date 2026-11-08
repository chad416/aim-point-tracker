/*
 * WebAssembly entry points for the browser demo (web/index.html): the spot
 * detector of aimtrk/ (at_detect_frame, the same code the tracker service
 * runs) on frames the page renders. Only this file is web-specific; aimtrk/
 * is compiled unchanged (scripts/build-web.sh).
 */
#include <stdint.h>
#include <string.h>

#include "aimtrk/detect.h"

#define WEB_EXPORT(name) __attribute__((export_name(name)))

#define MAX_W      1280
#define MAX_H      800
#define MAX_SPOTS  64
#define ARENA_SIZE (1u << 21) /* 2 MiB; at_detect_mem_size() is checked against it */

static uint8_t frame[MAX_W * MAX_H];
static uint8_t arena[ARENA_SIZE] __attribute__((aligned(16)));
static at_detect det;
static at_spot spots[MAX_SPOTS];
static double spot_out[MAX_SPOTS * 10];
static double stats_out[16];
static double cfg_out[8];

/* Returns the detector memory in bytes, or 0 on an invalid size / too little memory. */
WEB_EXPORT("web_init")
uint32_t web_init(int width, int height)
{
	at_detect_cfg cfg;

	if ((width <= 0) || (height <= 0) || (width > MAX_W) || (height > MAX_H)) {
		return 0;
	}
	at_detect_cfg_default(&cfg, (uint16_t)width, (uint16_t)height);
	size_t need = at_detect_mem_size(&cfg);

	if ((need == 0U) || (need > sizeof(arena)) ||
	    (at_detect_init(&det, &cfg, arena, sizeof(arena)) != 0)) {
		return 0;
	}
	cfg_out[0] = cfg.k_sigma;
	cfg_out[1] = cfg.t_min;
	cfg_out[2] = cfg.min_area;
	cfg_out[3] = cfg.max_area;
	cfg_out[4] = cfg.max_extent;
	cfg_out[5] = cfg.max_elong;
	cfg_out[6] = cfg.min_snr;
	cfg_out[7] = cfg.bg_shift;
	return (uint32_t)need;
}

WEB_EXPORT("web_cfg")
const double *web_cfg(void)
{
	return cfg_out;
}

/* The page writes each frame here (width * height bytes, stride = width). */
WEB_EXPORT("web_frame")
uint8_t *web_frame(void)
{
	return frame;
}

WEB_EXPORT("web_reset")
void web_reset(void)
{
	at_detect_reset(&det);
}

/*
 * Detect the spots in the frame. Per spot, 10 doubles: x, y, sum, peak, snr,
 * sig_major, sig_minor, area, n_sat, flags.
 */
WEB_EXPORT("web_detect")
int web_detect(void)
{
	int n = at_detect_frame(&det, frame, det.cfg.width, spots, MAX_SPOTS);

	for (int i = 0; i < n; i++) {
		double *o = &spot_out[10 * i];

		o[0] = spots[i].x;
		o[1] = spots[i].y;
		o[2] = spots[i].sum;
		o[3] = spots[i].peak;
		o[4] = spots[i].snr;
		o[5] = spots[i].sig_major;
		o[6] = spots[i].sig_minor;
		o[7] = spots[i].area;
		o[8] = spots[i].n_sat;
		o[9] = spots[i].flags;
	}
	return n;
}

WEB_EXPORT("web_spots")
const double *web_spots(void)
{
	return spot_out;
}

/*
 * The last frame's statistics: sigma, bg_mean, runs, blobs, spots, rejected
 * small, large, weak, elongated, frozen cells, snapped cells, recovered,
 * overflow, learning.
 */
WEB_EXPORT("web_stats")
const double *web_stats(void)
{
	const at_detect_stats *s = at_detect_get_stats(&det);

	stats_out[0] = s->sigma;
	stats_out[1] = s->bg_mean;
	stats_out[2] = s->runs;
	stats_out[3] = s->blobs;
	stats_out[4] = s->spots;
	for (int i = 0; i < AT_REJ_COUNT; i++) {
		stats_out[5 + i] = s->rejected[i];
	}
	stats_out[9] = s->frozen_cells;
	stats_out[10] = s->snapped_cells;
	stats_out[11] = s->recovered;
	stats_out[12] = s->overflow;
	stats_out[13] = s->learning ? 1.0 : 0.0;
	return stats_out;
}

/* Background estimate at a pixel, DN (for the magnifier). */
WEB_EXPORT("web_bg")
float web_bg(int x, int y)
{
	return at_detect_bg(&det, x, y);
}
