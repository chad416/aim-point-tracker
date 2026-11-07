/*
 * IR spot detection (SPEC F02, §6.1).
 *
 *  1. Background on a grid of AT_BG_CELL x AT_BG_CELL pixel cells: a running
 *     mean of each cell, in 16-bit fixed point (DN * 256, at most 65280), not
 *     updated under spots.
 *  2. Noise sigma from the median absolute deviation of (frame - background)
 *     on every 4th pixel of every 4th row.
 *  3. Threshold per cell: background + max(k_sigma * sigma, t_min).
 *  4. Runs of pixels above the threshold per row, joined into blobs across
 *     rows with a union-find (8-connected).
 *  5. Blobs rejected by area, extent, peak SNR and elongation; the rest are
 *     located by at_centroid() on the blob plus a one-pixel border.
 *
 * Memory: the caller provides one block of at_detect_mem_size() bytes at
 * init. Nothing is allocated afterwards, and the run, blob and spot tables
 * have fixed sizes (overflow is counted, not grown). The library does no I/O.
 */
#ifndef AIMTRK_DETECT_H_
#define AIMTRK_DETECT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aimtrk/centroid.h"

#define AT_BG_CELL 4 /* background cell size, px; image sizes must be multiples of it */

typedef struct {
	uint16_t width, height; /* px, multiples of AT_BG_CELL */
	uint8_t bg_shift;       /* running mean weight 2^-bg_shift per frame */
	float k_sigma;          /* threshold = background + max(k_sigma * sigma, t_min) */
	float t_min;            /* DN */
	uint16_t min_area;      /* px; smaller blobs are rejected (hot pixels) */
	uint16_t max_area;      /* px; larger blobs are rejected (reflections, a window) */
	uint16_t max_extent;    /* px, bounding box side; larger blobs are rejected */
	float max_elong;        /* sig_major / sig_minor; more elongated blobs are rejected */
	float min_snr;          /* peak SNR; weaker blobs are rejected */
	uint8_t sat_level;      /* DN at and above which a pixel is saturated */
	uint16_t max_runs;      /* run table size, per frame */
	uint16_t max_blobs;     /* blob table size, per frame */
} at_detect_cfg;

enum at_reject {
	AT_REJ_SMALL,
	AT_REJ_LARGE,
	AT_REJ_WEAK,
	AT_REJ_ELONGATED,
	AT_REJ_COUNT
};

/* at_detect_stats.overflow */
#define AT_OVF_RUNS  0x01U
#define AT_OVF_BLOBS 0x02U
#define AT_OVF_SPOTS 0x04U

typedef struct {
	float sigma;                     /* noise estimate, DN */
	float bg_mean;                   /* mean background, DN (ambient level) */
	uint32_t runs, blobs, spots;
	uint32_t rejected[AT_REJ_COUNT];
	uint32_t frozen_cells;           /* background cells not updated (under spots) */
	uint8_t overflow;                /* AT_OVF_* */
	bool learning;                   /* the frame seeded the background; nothing detected */
} at_detect_stats;

typedef struct {
	uint16_t x0, x1, y; /* pixels x0..x1 of row y */
	uint16_t label;     /* union-find parent while scanning, root afterwards */
} at_run;

typedef struct {
	uint16_t root;
	uint16_t x0, x1, y0, y1; /* bounding box */
	uint32_t area;
} at_blob;

typedef struct {
	at_detect_cfg cfg;
	int cw, ch;          /* background grid size, cells */
	uint16_t *bg;        /* cw*ch, DN * 256 */
	uint8_t *thr;        /* cw*ch, this frame's threshold: pixel > thr is detected */
	uint16_t *cellsum;   /* cw*ch, this frame's cell sums */
	uint8_t *freeze;     /* cw*ch, nonzero = under a spot, not updated */
	at_run *runs;        /* max_runs */
	uint16_t *blob_of;   /* max_runs: root run -> blob index */
	at_blob *blobs;      /* max_blobs */
	uint8_t *mask;       /* (max_extent + 2)^2 centroid window mask */
	float *bgw;          /* (max_extent + 2)^2 centroid window background */
	uint32_t *hist;      /* 511 bins, frame - background */
	bool bg_valid;
	at_detect_stats stats;
} at_detect;

/* Defaults for the reference lane (SPEC §2a). */
void at_detect_cfg_default(at_detect_cfg *cfg, uint16_t width, uint16_t height);

/* Bytes of memory at_detect_init() needs for this configuration (0: invalid configuration). */
size_t at_detect_mem_size(const at_detect_cfg *cfg);

/* Returns 0, or -1 for an invalid configuration or too little memory. */
int at_detect_init(at_detect *d, const at_detect_cfg *cfg, void *mem, size_t mem_size);

/* Seed the background from a frame (no spots expected). at_detect_frame() does this on its first call. */
void at_detect_learn(at_detect *d, const uint8_t *img, size_t stride);

/* Forget the background; the next frame seeds it again. */
void at_detect_reset(at_detect *d);

/*
 * Detect and locate the spots in a frame. Writes up to max_spots spots in
 * raster order of their first pixel, returns the number written.
 */
int at_detect_frame(at_detect *d, const uint8_t *img, size_t stride, at_spot *spots, int max_spots);

/* Noise sigma of a frame against the current background, DN (step 2). */
float at_detect_noise(at_detect *d, const uint8_t *img, size_t stride);

/* Background at a pixel, DN. */
float at_detect_bg(const at_detect *d, int x, int y);

const at_detect_stats *at_detect_get_stats(const at_detect *d);

#endif /* AIMTRK_DETECT_H_ */
