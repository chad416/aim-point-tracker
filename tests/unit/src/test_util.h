/*
 * Test helpers: a small deterministic PRNG, Gaussian noise, and spots rendered
 * as a 2-D Gaussian integrated exactly over each pixel (erf), so the true
 * centre of every test spot is known.
 */
#ifndef TEST_UTIL_H_
#define TEST_UTIL_H_

#include <math.h>
#include <stdint.h>
#include <string.h>

static inline uint32_t tu_rand(uint32_t *s)
{
	uint32_t x = *s;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*s = x;
	return x;
}

/* Uniform in [0, 1). */
static inline double tu_unif(uint32_t *s)
{
	return (double)(tu_rand(s) >> 8) / 16777216.0;
}

/* Standard normal (Box-Muller). */
static inline double tu_randn(uint32_t *s)
{
	double u1 = tu_unif(s) + 1e-12;
	double u2 = tu_unif(s);

	return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

/* Fraction of a 1-D Gaussian (centre c, size sigma) falling in pixel x (x - 0.5 .. x + 0.5). */
static inline double tu_pix(int x, double c, double sigma)
{
	double k = 1.0 / (sigma * 1.4142135623730951);

	return 0.5 * (erf(((double)x + 0.5 - c) * k) - erf(((double)x - 0.5 - c) * k));
}

/* Add an axis-aligned Gaussian spot with the given peak (of the continuous Gaussian) to sig. */
static inline void tu_spot(float *sig, int w, int h, double u, double v, double sx, double sy,
			   double peak)
{
	double total = peak * 6.283185307179586 * sx * sy;

	for (int y = 0; y < h; y++) {
		double fy = tu_pix(y, v, sy);

		if (fy < 1e-9) {
			continue;
		}
		for (int x = 0; x < w; x++) {
			sig[(y * w) + x] += (float)(total * fy * tu_pix(x, u, sx));
		}
	}
}

static inline void tu_fill(float *sig, int n, float level)
{
	for (int i = 0; i < n; i++) {
		sig[i] = level;
	}
}

/* Round to 8 bits with optional Gaussian noise; clip to 0..255. */
static inline void tu_quantise(const float *sig, uint8_t *img, int n, double noise, uint32_t *seed)
{
	for (int i = 0; i < n; i++) {
		double v = (double)sig[i] + ((noise > 0.0) ? noise * tu_randn(seed) : 0.0);

		v = floor(v + 0.5);
		img[i] = (v < 0.0) ? 0U : ((v > 255.0) ? 255U : (uint8_t)v);
	}
}

#endif /* TEST_UTIL_H_ */
