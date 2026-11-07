/*
 * Sub-pixel spot location (SPEC F03, §6.2).
 *
 * Image coordinates: x to the right, y down, pixel centres at integer
 * coordinates (pixel (0, 0) covers -0.5..0.5 in x and y).
 *
 * The centroid is the intensity-weighted mean of the background-subtracted
 * pixels in a window, weights clipped at 0. A pixel mask restricts it to the
 * blob plus a one-pixel border. Saturated spots are flagged, and located
 * again from their unsaturated rim by a least-squares fit of a 2-D Gaussian
 * to the logarithm of the rim pixels.
 */
#ifndef AIMTRK_CENTROID_H_
#define AIMTRK_CENTROID_H_

#include <stddef.h>
#include <stdint.h>

/* at_spot.flags */
#define AT_SPOT_SATURATED  0x01U /* at least one pixel at or above the saturation level */
#define AT_SPOT_RIM        0x02U /* position from the rim fit, not the centroid */
#define AT_SPOT_RIM_FAILED 0x04U /* saturated, rim fit not usable: position is the centroid */
#define AT_SPOT_EDGE       0x08U /* window cut by the image border */

typedef struct {
	float x, y;        /* position, px (image coordinates) */
	float cx, cy;      /* plain weighted centroid, px (equals x, y unless AT_SPOT_RIM) */
	float sum;         /* integrated signal above background, DN */
	float peak;        /* largest pixel above background, DN */
	float snr;         /* peak / noise sigma */
	float sig_major;   /* spot size along its major axis (second moments), px */
	float sig_minor;   /* spot size along its minor axis, px */
	uint16_t area;     /* pixels above the detection threshold (filled in by detect) */
	uint16_t n_sat;    /* saturated pixels */
	uint8_t flags;
} at_spot;

/* A window of a frame, with an optional mask and per-pixel background. */
typedef struct {
	const uint8_t *px;   /* first pixel of the window */
	size_t stride;       /* bytes from one image row to the next */
	int x0, y0;          /* window origin in image coordinates */
	int w, h;            /* window size, px */
	const uint8_t *mask; /* w*h, row-major; nonzero = use the pixel. NULL: whole window */
	const float *bg;     /* w*h background, DN, row-major. NULL: bg_const everywhere */
	float bg_const;
} at_cwin;

/*
 * Locate the spot in the window. sigma is the frame's noise level (DN, used
 * for the SNR and the rim selection), sat_level the pixel value at and above
 * which a pixel counts as saturated. Returns 0, or -1 if the window holds no
 * signal above background (spot untouched).
 */
int at_centroid(const at_cwin *win, float sigma, uint8_t sat_level, at_spot *spot);

/*
 * Rim fit alone: fit ln(I - bg) = a + b x + c y + d x^2 + e xy + f y^2 to the
 * masked pixels below sat_level with signal above min_signal, weighted by the
 * signal squared, and return the Gaussian's centre. Returns 0, or -1 if there
 * are too few pixels or the fit is not a peak (spot position untouched).
 */
int at_centroid_rim(const at_cwin *win, uint8_t sat_level, float min_signal, float *x, float *y);

#endif /* AIMTRK_CENTROID_H_ */
