/*
 * Image <-> screen geometry (SPEC F04, §6.3).
 *
 *   pixel (u, v)
 *     -> distorted normalised  xd = (u - cx) / f,  yd = (v - cy) / f
 *     -> undistorted (Brown-Conrady k1, k2, p1, p2, inverted by Newton iteration)
 *     -> screen (x, y) in mm through a homography
 *
 * Screen coordinates: x right, y up, mm, origin at the screen centre.
 * f, cx and cy are the lens's nominal values; they are not fitted. The
 * homography is general (8 degrees of freedom), so it absorbs an error in the
 * nominal focal length and principal point (SPEC §6.4).
 */
#ifndef AIMTRK_GEOM_H_
#define AIMTRK_GEOM_H_

#include <stdbool.h>

typedef struct {
	double f;      /* px, focal length (fx = fy) */
	double cx, cy; /* px, principal point */
	double k1, k2; /* radial distortion */
	double p1, p2; /* tangential distortion */
} at_lens;

typedef struct {
	double m[9]; /* row-major 3 x 3 */
} at_h33;

typedef struct {
	at_lens lens;
	at_h33 to_cam;    /* screen mm -> undistorted normalised camera coordinates */
	at_h33 to_screen; /* the inverse */
	double half_w;    /* screen half width, mm */
	double half_h;    /* screen half height, mm */
} at_geom;

/* at_geom_to_screen() results */
#define AT_GEOM_ON_SCREEN  0
#define AT_GEOM_OFF_SCREEN 1
#define AT_GEOM_FAIL       (-1)

/* Distort an undistorted normalised point (the forward lens model). */
void at_lens_distort(const at_lens *l, double xn, double yn, double *xd, double *yd);

/*
 * Invert at_lens_distort() by Newton iteration. False if it does not converge
 * to 1e-12, or converges where the model folds over (not the physical inverse).
 */
bool at_lens_undistort(const at_lens *l, double xd, double yd, double *xn, double *yn);

/* Apply a homography. False if the point maps to infinity. */
bool at_h33_apply(const at_h33 *h, double x, double y, double *ox, double *oy);

/* Inverse, scaled so that m[8] = 1 where possible. False if singular. */
bool at_h33_invert(const at_h33 *h, at_h33 *inv);

/*
 * Homography from n >= 4 point pairs, src[2i], src[2i + 1] -> dst[2i], dst[2i + 1],
 * by the normalised direct linear transform (Hartley): both sets are moved to
 * their centroid and scaled to a mean distance of sqrt(2), the 2n x 9 system
 * is solved for its smallest singular vector, and the result is denormalised.
 * Returns 0, or -1 for fewer than 4 points or a degenerate set.
 */
int at_h33_dlt(const double *src, const double *dst, int n, at_h33 *h);

/* Set up a geometry from a lens, the screen -> camera homography and the screen size (mm). */
int at_geom_init(at_geom *g, const at_lens *l, const at_h33 *to_cam, double screen_w, double screen_h);

/* Pixel -> screen mm. Returns AT_GEOM_ON_SCREEN, AT_GEOM_OFF_SCREEN (point still valid) or AT_GEOM_FAIL. */
int at_geom_to_screen(const at_geom *g, double u, double v, double *x_mm, double *y_mm);

/* Screen mm -> pixel. False if the point is behind the camera model (maps to infinity). */
bool at_geom_to_image(const at_geom *g, double x_mm, double y_mm, double *u, double *v);

#endif /* AIMTRK_GEOM_H_ */
