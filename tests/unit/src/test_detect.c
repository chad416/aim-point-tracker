/*
 * SPEC F02, §6.1: background, noise estimate, threshold, runs and union-find,
 * rejection rules, background freeze under spots, table limits.
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "aimtrk/detect.h"
#include "test_util.h"

#define W  64
#define H  64
#define NW 128 /* noise tests use a larger frame for more samples */
#define NH 128

static uint8_t mem[64 * 1024];
static at_detect det;
static at_detect_cfg cfg;
static float sig[NH * NW];
static uint8_t img[NH * NW];
static at_spot spots[8];

static void setup(uint16_t w, uint16_t h)
{
	at_detect_cfg_default(&cfg, w, h);
	cfg.max_runs = 1024; /* small test frames: keep the memory block small */
}

static void start(void)
{
	zassert_true(at_detect_mem_size(&cfg) <= sizeof(mem));
	zassert_ok(at_detect_init(&det, &cfg, mem, sizeof(mem)));
}

/* A flat frame at `level` (DN), learned as the background. */
static void flat_background(float level)
{
	tu_fill(sig, W * H, level);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	at_detect_learn(&det, img, W);
}

static void before(void *f)
{
	ARG_UNUSED(f);
	setup(W, H);
	start();
}

ZTEST_SUITE(detect, NULL, NULL, before, NULL, NULL);

ZTEST(detect, test_config_and_memory)
{
	at_detect_cfg bad = cfg;

	bad.width = 66; /* not a multiple of the cell size */
	zassert_equal(at_detect_mem_size(&bad), 0U);
	zassert_equal(at_detect_init(&det, &bad, mem, sizeof(mem)), -1);
	bad = cfg;
	bad.max_extent = 255;
	zassert_equal(at_detect_mem_size(&bad), 0U);
	zassert_equal(at_detect_init(&det, &cfg, mem, at_detect_mem_size(&cfg) - 1U), -1);
	zassert_equal(at_detect_init(&det, &cfg, NULL, sizeof(mem)), -1);
	/* An unaligned block works: init aligns it. */
	zassert_ok(at_detect_init(&det, &cfg, mem + 3, sizeof(mem) - 3U));

	/* The full reference frame: memory is fixed, and modest. */
	at_detect_cfg full;

	at_detect_cfg_default(&full, 1280, 800);
	TC_PRINT("1280 x 800: %u bytes\n", (unsigned int)at_detect_mem_size(&full));
	zassert_true(at_detect_mem_size(&full) < (512U * 1024U));
}

ZTEST(detect, test_first_frame_learns_then_reset)
{
	tu_fill(sig, W * H, 20.0F);
	tu_spot(sig, W, H, 30.0, 30.0, 1.2, 1.2, 150.0);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_true(at_detect_get_stats(&det)->learning);

	flat_background(20.0F);
	tu_fill(sig, W * H, 20.0F);
	tu_spot(sig, W, H, 30.0, 30.0, 1.2, 1.2, 150.0);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 1);
	zassert_false(at_detect_get_stats(&det)->learning);

	at_detect_reset(&det);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_true(at_detect_get_stats(&det)->learning);
}

/* MAD noise estimate on Gaussian noise (plus 1/12 DN^2 of quantisation). */
ZTEST(detect, test_noise_estimate)
{
	static const double levels[] = {1.0, 2.0, 5.0};
	uint32_t seed = 12345U;

	setup(NW, NH);
	start();
	for (size_t i = 0; i < ARRAY_SIZE(levels); i++) {
		tu_fill(sig, NW * NH, 40.0F);
		tu_quantise(sig, img, NW * NH, 0.0, NULL);
		at_detect_learn(&det, img, NW);
		tu_quantise(sig, img, NW * NH, levels[i], &seed);
		float s = at_detect_noise(&det, img, NW);
		double expect = sqrt((levels[i] * levels[i]) + (1.0 / 12.0));

		TC_PRINT("noise %.1f DN: estimate %.3f\n", levels[i], (double)s);
		/* MAD on 8-bit data: +9 % at 1 DN, within a few % from 2 DN up. */
		zassert_within((double)s, expect, 0.10 * expect);
	}
	/* Noise-free and flat: the estimate is below one DN (the quantisation floor). */
	tu_quantise(sig, img, NW * NH, 0.0, NULL);
	zassert_true(at_detect_noise(&det, img, NW) < 0.5F);
}

/* With little noise, t_min sets the threshold: pixel > background + 8 DN. */
ZTEST(detect, test_threshold_t_min)
{
	flat_background(20.0F);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	for (int y = 20; y < 22; y++) {
		for (int x = 20; x < 22; x++) {
			img[(y * W) + x] = 28; /* = threshold: not detected */
		}
	}
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_equal(at_detect_get_stats(&det)->blobs, 0U);

	flat_background(20.0F);
	for (int y = 20; y < 22; y++) {
		for (int x = 20; x < 22; x++) {
			img[(y * W) + x] = 29;
		}
	}
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 1);
	zassert_within(spots[0].x, 20.5F, 1e-5F);
	zassert_within(spots[0].y, 20.5F, 1e-5F);
	zassert_equal(spots[0].area, 4U);
}

/* With noise, k_sigma * sigma sets it: a 15 DN spot is below 6 sigma at 3 DN noise, a 40 DN one above. */
ZTEST(detect, test_threshold_k_sigma)
{
	uint32_t seed = 99U;

	flat_background(30.0F);
	tu_fill(sig, W * H, 30.0F);
	tu_spot(sig, W, H, 20.3, 40.6, 1.2, 1.2, 15.0);
	tu_spot(sig, W, H, 44.2, 18.8, 1.2, 1.2, 40.0);
	tu_quantise(sig, img, W * H, 3.0, &seed);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 1);
	zassert_within(at_detect_get_stats(&det)->sigma, 3.0F, 0.4F);
	zassert_within(spots[0].x, 44.2F, 0.3F);
	zassert_within(spots[0].y, 18.8F, 0.3F);
}

/* Label count for a binary pattern drawn at 100 DN on a 20 DN background. */
static uint32_t blobs_of(const char *const rows[], int nrows)
{
	flat_background(20.0F);
	for (int r = 0; r < nrows; r++) {
		for (int c = 0; rows[r][c] != '\0'; c++) {
			if (rows[r][c] == '#') {
				img[((10 + r) * W) + 10 + c] = 100;
			}
		}
	}
	(void)at_detect_frame(&det, img, W, spots, 8);
	return at_detect_get_stats(&det)->blobs;
}

ZTEST(detect, test_union_find_shapes)
{
	static const char *const u_shape[] = {
		"#....#",
		"#....#",
		"#....#",
		"######",
	};
	static const char *const w_shape[] = {
		"#...#...#",
		"#...#...#",
		".#.#.#.#.",
		"..#...#..",
		"...###...",
	};
	static const char *const diagonal[] = {
		"#....",
		".#...",
		"..#..",
		"...#.",
		"....#",
	};
	static const char *const anti_diagonal[] = {
		"....#",
		"...#.",
		"..#..",
		".#...",
		"#....",
	};
	static const char *const apart[] = {
		"##.##",
		"##.##",
	};
	static const char *const spiral[] = {
		"#######",
		"#.....#",
		"#.###.#",
		"#.#.#.#",
		"#.#...#",
		"#.#####",
	};

	zassert_equal(blobs_of(u_shape, 4), 1U);
	zassert_equal(blobs_of(w_shape, 5), 1U);
	zassert_equal(blobs_of(diagonal, 5), 1U);
	zassert_equal(blobs_of(anti_diagonal, 5), 1U);
	zassert_equal(blobs_of(apart, 2), 2U);
	zassert_equal(blobs_of(spiral, 6), 1U);
}

ZTEST(detect, test_rejection_rules)
{
	const at_detect_stats *st = at_detect_get_stats(&det);

	/* Two pixels: a hot pixel pair. */
	flat_background(20.0F);
	img[(30 * W) + 30] = 200;
	img[(30 * W) + 31] = 200;
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_equal(st->rejected[AT_REJ_SMALL], 1U);

	/* Wider than max_extent (24): a reflection. */
	flat_background(20.0F);
	memset(&img[30 * W + 5], 120, 30);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_equal(st->rejected[AT_REJ_LARGE], 1U);

	/* 20 x 20: inside max_extent but more than max_area (300). */
	flat_background(20.0F);
	for (int y = 10; y < 30; y++) {
		memset(&img[(y * W) + 10], 120, 20);
	}
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_equal(st->rejected[AT_REJ_LARGE], 1U);

	/* A one-pixel line: a glint. */
	flat_background(20.0F);
	memset(&img[30 * W + 10], 120, 16);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_equal(st->rejected[AT_REJ_ELONGATED], 1U);

	/* Above the threshold but below min_snr. */
	cfg.min_snr = 100.0F;
	start();
	flat_background(20.0F);
	tu_fill(sig, W * H, 20.0F);
	tu_spot(sig, W, H, 30.0, 30.0, 1.2, 1.2, 30.0);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_equal(st->rejected[AT_REJ_WEAK], 1U);
}

/* Spots come out in raster order of their first pixel, with their quality figures. */
ZTEST(detect, test_order_and_quality)
{
	flat_background(20.0F);
	tu_fill(sig, W * H, 20.0F);
	tu_spot(sig, W, H, 12.4, 40.7, 1.2, 1.2, 150.0);
	tu_spot(sig, W, H, 50.1, 15.2, 1.2, 1.2, 100.0);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 2);
	zassert_within(spots[0].y, 15.2F, 0.02F);
	zassert_within(spots[1].y, 40.7F, 0.02F);
	zassert_within(spots[1].x, 12.4F, 0.02F);
	zassert_true((spots[1].peak > 80.0F) && (spots[1].peak < 150.0F), "pixel-integrated peak %f",
		     (double)spots[1].peak);
	zassert_true(spots[1].area >= 9U);
	zassert_equal(spots[1].flags, 0U);
	zassert_within(spots[1].sig_major / spots[1].sig_minor, 1.0F, 0.1F);
}

/* A spot cut by the image border is still located, and flagged. */
ZTEST(detect, test_edge_flag)
{
	flat_background(20.0F);
	tu_fill(sig, W * H, 20.0F);
	tu_spot(sig, W, H, 0.4, 30.0, 1.2, 1.2, 150.0);
	tu_spot(sig, W, H, 32.0, 63.0, 1.2, 1.2, 150.0);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 2);
	for (int i = 0; i < 2; i++) {
		zassert_true((spots[i].flags & AT_SPOT_EDGE) != 0U);
	}
	/* Half the spot is outside: the centroid is pulled inward, which is why the flag exists. */
	TC_PRINT("edge spots: x %.3f (true 0.4), y %.3f (true 63.0)\n", (double)spots[0].x,
		 (double)spots[1].y);
	zassert_true((spots[0].x > 0.4F) && (spots[0].x < 1.5F));
	zassert_true((spots[1].y < 63.0F) && (spots[1].y > 61.5F));
}

/*
 * The background follows a slow change of the ambient level and is frozen
 * under a spot. The spot is there in every other frame, as an aim strobe with
 * two time slots (SPEC §6.5), so the cells under it still update in between.
 */
ZTEST(detect, test_background_tracks_and_freezes)
{
	const at_detect_stats *st = at_detect_get_stats(&det);
	float level = 20.0F;

	flat_background(level);
	for (int k = 0; k < 240; k++) {
		bool spot = (k % 2) == 0;

		if (k < 100) {
			level += 0.1F; /* 20 -> 30 DN in one second, then 1.4 s steady */
		}
		tu_fill(sig, W * H, level);
		if (spot) {
			tu_spot(sig, W, H, 30.3, 30.6, 1.2, 1.2, 150.0);
		}
		tu_quantise(sig, img, W * H, 0.0, NULL);
		zassert_equal(at_detect_frame(&det, img, W, spots, 8), spot ? 1 : 0, "frame %d", k);
	}
	/* Left: running-mean lag plus the truncation stall (under 1/16 DN). */
	zassert_within(at_detect_bg(&det, 5, 5), 30.0F, 0.1F, "%f", (double)at_detect_bg(&det, 5, 5));
	zassert_within(at_detect_bg(&det, 30, 30), 30.0F, 0.15F, "%f", (double)at_detect_bg(&det, 30, 30));
	zassert_within(st->bg_mean, 30.0F, 0.1F);

	/* One more frame with the spot, one DN brighter: only the cells away from it move. */
	float far = at_detect_bg(&det, 5, 5);
	float under = at_detect_bg(&det, 30, 30);

	tu_fill(sig, W * H, 31.0F);
	tu_spot(sig, W, H, 30.3, 30.6, 1.2, 1.2, 150.0);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 1);
	zassert_equal(at_detect_bg(&det, 30, 30), under, "frozen under the spot");
	zassert_true(at_detect_bg(&det, 5, 5) > far);
	zassert_true(st->frozen_cells >= 9U);
	zassert_within(spots[0].x, 30.3F, 0.05F);
}

ZTEST(detect, test_table_limits)
{
	static uint8_t three[W * H];
	const at_detect_stats *st = at_detect_get_stats(&det);

	tu_fill(sig, W * H, 20.0F);
	tu_spot(sig, W, H, 10.0, 10.0, 1.2, 1.2, 150.0);
	tu_spot(sig, W, H, 30.0, 30.0, 1.2, 1.2, 150.0);
	tu_spot(sig, W, H, 50.0, 50.0, 1.2, 1.2, 150.0);
	tu_quantise(sig, three, W * H, 0.0, NULL);

	/* Three spots, one spot slot: the first is returned, the overflow counted. */
	flat_background(20.0F);
	zassert_equal(at_detect_frame(&det, three, W, spots, 1), 1);
	zassert_equal(st->overflow, AT_OVF_SPOTS);
	zassert_within(spots[0].y, 10.0F, 0.05F);

	/* Two blob slots: the third blob is dropped. */
	cfg.max_blobs = 2;
	start();
	flat_background(20.0F);
	zassert_equal(at_detect_frame(&det, three, W, spots, 8), 2);
	zassert_equal(st->overflow, AT_OVF_BLOBS);

	/*
	 * Blobs too small to be a spot take no slot: still two slots, and a spot
	 * after forty 2-pixel specks in raster order is found (M4: a scene
	 * change about one threshold high breaks up into hundreds of them).
	 */
	static uint8_t specks[W * H];

	tu_fill(sig, W * H, 20.0F);
	for (int i = 0; i < 40; i++) {
		int x = 2 + ((i % 20) * 3);
		int y = 2 + ((i / 20) * 3);

		sig[(y * W) + x] = 60.0F;
		sig[(y * W) + x + 1] = 60.0F;
	}
	tu_spot(sig, W, H, 40.0, 40.0, 1.2, 1.2, 150.0);
	tu_quantise(sig, specks, W * H, 0.0, NULL);
	flat_background(20.0F);
	zassert_equal(at_detect_frame(&det, specks, W, spots, 8), 1);
	zassert_equal(st->overflow, 0U);
	zassert_equal(st->rejected[AT_REJ_SMALL], 40U);
	zassert_equal(st->blobs, 1U);
	zassert_within(spots[0].x, 40.0F, 0.05F);

	/* Four run slots: the scan stops adding runs. */
	cfg.max_blobs = 256;
	cfg.max_runs = 4;
	start();
	flat_background(20.0F);
	(void)at_detect_frame(&det, three, W, spots, 8);
	zassert_true((st->overflow & AT_OVF_RUNS) != 0U);
	zassert_equal(st->runs, 4U);
}

/* Random sub-pixel positions at the nominal spot (peak 150 DN) with 1 DN noise. */
ZTEST(detect, test_accuracy_nominal)
{
	uint32_t seed = 7U;
	double se = 0.0;
	int n = 200;

	for (int i = 0; i < n; i++) {
		double u = 31.5 + (tu_unif(&seed) - 0.5);
		double v = 31.5 + (tu_unif(&seed) - 0.5);

		flat_background(16.0F);
		tu_fill(sig, W * H, 16.0F);
		tu_spot(sig, W, H, u, v, 1.2, 1.2, 150.0);
		tu_quantise(sig, img, W * H, 1.0, &seed);
		zassert_equal(at_detect_frame(&det, img, W, spots, 8), 1);
		se += pow((double)spots[0].x - u, 2.0) + pow((double)spots[0].y - v, 2.0);
	}
	double rms = sqrt(se / n);

	TC_PRINT("nominal spot, 1 DN noise: radial RMS %.4f px\n", rms);
	zassert_true(rms < 0.05, "%f", rms);
}

/*
 * Step 6: a spot on top of a scene change (a projected shape moving in) merges
 * with it into one large blob; it is found inside, against the blob's level.
 */
ZTEST(detect, test_spot_on_scene_change)
{
	const at_detect_stats *st = at_detect_get_stats(&det);

	flat_background(20.0F);
	tu_fill(sig, W * H, 20.0F);
	for (int y = 8; y < 56; y++) {
		for (int x = 8; x < 56; x++) {
			sig[(y * W) + x] += 15.0F; /* the change: 15 DN, over the 8 DN threshold */
		}
	}
	tu_spot(sig, W, H, 30.4, 25.7, 1.2, 1.2, 150.0);
	tu_spot(sig, W, H, 44.2, 40.6, 1.2, 1.2, 100.0);
	tu_quantise(sig, img, W * H, 0.0, NULL);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 2);
	zassert_equal(st->rejected[AT_REJ_LARGE], 1U);
	zassert_equal(st->recovered, 2U);
	for (int i = 0; i < 2; i++) {
		zassert_true((spots[i].flags & AT_SPOT_ON_CHANGE) != 0U);
	}
	zassert_within(spots[0].x, 30.4F, 0.05F, "%f", (double)spots[0].x);
	zassert_within(spots[0].y, 25.7F, 0.05F, "%f", (double)spots[0].y);
	zassert_within(spots[1].x, 44.2F, 0.05F, "%f", (double)spots[1].x);
	zassert_within(spots[1].y, 40.6F, 0.05F, "%f", (double)spots[1].y);

	/* The change alone gives no spot; neither does a saturated area (nothing above its level). */
	flat_background(20.0F);
	tu_fill(sig, W * H, 20.0F);
	for (int y = 8; y < 56; y++) {
		for (int x = 8; x < 56; x++) {
			sig[(y * W) + x] = ((x < 32) ? 35.0F : 300.0F);
		}
	}
	tu_quantise(sig, img, W * H, 0.0, NULL);
	zassert_equal(at_detect_frame(&det, img, W, spots, 8), 0);
	zassert_equal(st->recovered, 0U);
}
