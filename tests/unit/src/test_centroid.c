/*
 * SPEC F03, §6.2: sub-pixel centroid, moments, SNR, saturation and the rim fit.
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "aimtrk/centroid.h"
#include "test_util.h"

#define W 15
#define H 15

ZTEST_SUITE(centroid, NULL, NULL, NULL, NULL, NULL);

static uint8_t img[H * W];
static float sig[H * W];

static at_cwin whole(float bg)
{
	return (at_cwin){.px = img, .stride = W, .x0 = 100, .y0 = 200, .w = W, .h = H,
			 .mask = NULL, .bg = NULL, .bg_const = bg};
}

/* Weights 1 and 3 above background in one row: centroid 1.75 / 4 of the way, exactly. */
ZTEST(centroid, test_weighted_mean_exact)
{
	at_spot s;
	at_cwin win = whole(10.0F);

	memset(img, 10, sizeof(img));
	img[(7 * W) + 1] = 11;
	img[(7 * W) + 2] = 13;
	zassert_ok(at_centroid(&win, 1.0F, 255, &s));
	zassert_within(s.x, 100.0F + 1.75F, 1e-5F);
	zassert_within(s.y, 207.0F, 1e-5F);
	zassert_within(s.sum, 4.0F, 1e-5F);
	zassert_within(s.peak, 3.0F, 1e-5F);
	zassert_within(s.snr, 3.0F, 1e-5F);
	zassert_equal(s.flags, 0U);
	zassert_equal(s.x, s.cx);
}

/* Pixels outside the mask and pixels below background do not move the centroid. */
ZTEST(centroid, test_mask_and_clipping)
{
	static uint8_t mask[H * W];
	at_spot a, b;
	at_cwin win = whole(10.0F);

	memset(img, 10, sizeof(img));
	img[(5 * W) + 5] = 30;
	img[(5 * W) + 6] = 20;
	zassert_ok(at_centroid(&win, 1.0F, 255, &a));

	memset(mask, 0, sizeof(mask));
	for (int y = 4; y <= 6; y++) {
		for (int x = 4; x <= 7; x++) {
			mask[(y * W) + x] = 1;
		}
	}
	img[(12 * W) + 12] = 250; /* bright, outside the mask */
	img[(4 * W) + 7] = 0;     /* below background, inside the mask */
	win.mask = mask;
	zassert_ok(at_centroid(&win, 1.0F, 255, &b));
	zassert_within(b.x, a.x, 1e-5F);
	zassert_within(b.y, a.y, 1e-5F);
	zassert_within(b.sum, a.sum, 1e-5F);
}

ZTEST(centroid, test_no_signal)
{
	at_spot s = {.x = -1.0F};
	at_cwin win = whole(10.0F);

	memset(img, 9, sizeof(img));
	zassert_equal(at_centroid(&win, 1.0F, 255, &s), -1);
	zassert_equal(s.x, -1.0F, "spot must be left untouched");
}

/* The per-pixel background is used where given; the SNR uses a 0.5 DN noise floor. */
ZTEST(centroid, test_per_pixel_background_and_snr_floor)
{
	static float bg[H * W];
	at_spot s;
	at_cwin win = whole(0.0F);

	for (int i = 0; i < (H * W); i++) {
		bg[i] = (float)(i % W); /* a ramp across the window */
		img[i] = (uint8_t)(i % W);
	}
	img[(3 * W) + 9] = 9 + 6;
	win.bg = bg;
	zassert_ok(at_centroid(&win, 0.1F, 255, &s));
	zassert_within(s.x, 109.0F, 1e-5F);
	zassert_within(s.y, 203.0F, 1e-5F);
	zassert_within(s.snr, 12.0F, 1e-4F, "6 DN over the 0.5 DN floor");
}

/* Noise-free 8-bit spots over a grid of sub-pixel phases: bias from quantisation only. */
ZTEST(centroid, test_subpixel_phases)
{
	float worst = 0.0F;

	for (int i = 0; i <= 10; i++) {
		for (int j = 0; j <= 10; j++) {
			double u = 7.0 + ((i - 5) * 0.1);
			double v = 7.0 + ((j - 5) * 0.1);
			at_spot s;
			at_cwin win = whole(20.0F);

			tu_fill(sig, H * W, 20.0F);
			tu_spot(sig, W, H, u, v, 1.2, 1.2, 150.0);
			tu_quantise(sig, img, H * W, 0.0, NULL);
			zassert_ok(at_centroid(&win, 0.7F, 255, &s));
			float e = hypotf(s.x - (float)(100.0 + u), s.y - (float)(200.0 + v));

			worst = fmaxf(worst, e);
		}
	}
	TC_PRINT("worst phase error %.4f px\n", (double)worst);
	zassert_true(worst < 0.02F, "%f", (double)worst); /* a fifth of the 0.1 px budget */
}

/* Second moments give the spot's size along its axes. */
ZTEST(centroid, test_moments)
{
	at_spot s;
	at_cwin win = whole(10.0F);

	tu_fill(sig, H * W, 10.0F);
	tu_spot(sig, W, H, 7.2, 6.9, 2.0, 1.0, 200.0);
	tu_quantise(sig, img, H * W, 0.0, NULL);
	zassert_ok(at_centroid(&win, 0.7F, 255, &s));
	/* A pixel adds 1/12 px^2 of variance to a sampled Gaussian. */
	zassert_within(s.sig_major, sqrtf(4.0F + (1.0F / 12.0F)), 0.03F, "%f", (double)s.sig_major);
	zassert_within(s.sig_minor, sqrtf(1.0F + (1.0F / 12.0F)), 0.03F, "%f", (double)s.sig_minor);
	/* Integrated signal: within 1 % (8-bit rounding of the tails, the window edge). */
	zassert_within(s.sum, (float)(200.0 * 6.283185307179586 * 2.0), 25.0F, "%f", (double)s.sum);
}

/* Saturated spots: flagged, located by the rim fit, better than the clipped centroid. */
ZTEST(centroid, test_saturated_rim_fit)
{
	float worst_rim = 0.0F;
	float worst_plain = 0.0F;

	for (int i = 0; i <= 6; i++) {
		for (int j = 0; j <= 6; j++) {
			double u = 7.0 + ((i - 3) * 0.15);
			double v = 7.0 + ((j - 3) * 0.15);
			at_spot s;
			at_cwin win = whole(20.0F);

			tu_fill(sig, H * W, 20.0F);
			tu_spot(sig, W, H, u, v, 1.2, 1.2, 2000.0); /* 8x full scale */
			tu_quantise(sig, img, H * W, 0.0, NULL);
			zassert_ok(at_centroid(&win, 0.7F, 255, &s));
			zassert_equal(s.flags, AT_SPOT_SATURATED | AT_SPOT_RIM);
			zassert_true(s.n_sat >= 9U);
			worst_rim = fmaxf(worst_rim, hypotf(s.x - (float)(100.0 + u), s.y - (float)(200.0 + v)));
			worst_plain = fmaxf(worst_plain,
					    hypotf(s.cx - (float)(100.0 + u), s.cy - (float)(200.0 + v)));
		}
	}
	TC_PRINT("saturated: worst rim %.4f px, worst plain centroid %.4f px\n", (double)worst_rim,
		 (double)worst_plain);
	zassert_true(worst_rim < 0.02F, "%f", (double)worst_rim);
	zassert_true(worst_rim < worst_plain);
}

/* A spot saturated across the whole window has no rim: the centroid is kept and flagged. */
ZTEST(centroid, test_rim_failed)
{
	at_spot s;
	at_cwin win = whole(20.0F);

	memset(img, 255, sizeof(img));
	zassert_ok(at_centroid(&win, 0.7F, 255, &s));
	zassert_equal(s.flags, AT_SPOT_SATURATED | AT_SPOT_RIM_FAILED);
	zassert_within(s.x, 107.0F, 1e-5F);
	zassert_equal(s.n_sat, (uint16_t)(W * H));
}

/* The rim fit refuses data that is not a peak, and degenerate pixel sets. */
ZTEST(centroid, test_rim_rejects)
{
	float x = -1.0F, y = -1.0F;
	at_cwin win = whole(10.0F);

	/* A bowl: ln(s) convex. */
	for (int r = 0; r < H; r++) {
		for (int c = 0; c < W; c++) {
			int d2 = ((c - 7) * (c - 7)) + ((r - 7) * (r - 7));

			img[(r * W) + c] = (uint8_t)(20 + (d2 / 2));
		}
	}
	zassert_equal(at_centroid_rim(&win, 255, 2.0F, &x, &y), -1);

	/* All rim pixels in one row: the normal equations are singular. */
	memset(img, 10, sizeof(img));
	for (int c = 2; c < 13; c++) {
		img[(7 * W) + c] = (uint8_t)(40 + (10 * (c % 3)));
	}
	zassert_equal(at_centroid_rim(&win, 255, 2.0F, &x, &y), -1);

	/* Too few pixels. */
	memset(img, 10, sizeof(img));
	img[(7 * W) + 7] = 100;
	zassert_equal(at_centroid_rim(&win, 255, 2.0F, &x, &y), -1);
	zassert_equal(x, -1.0F);

	/* A peak far outside the window: the edge of a much larger spot. */
	tu_fill(sig, H * W, 10.0F);
	tu_spot(sig, W, H, 40.0, 7.0, 12.0, 12.0, 240.0);
	tu_quantise(sig, img, H * W, 0.0, NULL);
	zassert_equal(at_centroid_rim(&win, 255, 2.0F, &x, &y), -1);
}
