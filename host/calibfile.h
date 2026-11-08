/*
 * Read calib.json (written by tools/calib/calibrate.py, SPEC F05).
 *
 * The file's own crc32 covers the whole JSON body in Python's canonical form,
 * which C does not reproduce; aim_host checks geometry_crc32 instead: CRC-32
 * (zlib) of the 18 numbers it uses (lens f, cx, cy, k1, k2, p1, p2; to_cam
 * m[0..8]; screen_w, screen_h) as little-endian IEEE-754 doubles. Python's
 * JSON numbers round-trip exactly through strtod, so both sides agree.
 * This is a reader for that one file layout, not a general JSON parser.
 */
#ifndef AIM_HOST_CALIBFILE_H_
#define AIM_HOST_CALIBFILE_H_

#include <stddef.h>
#include <stdint.h>

#include "aimtrk/geom.h"

uint32_t crc32_le(uint32_t crc, const uint8_t *p, size_t n);

/* 0, or -1 with a reason in err. */
int calib_load(const char *path, at_geom *g, int64_t *created_unix, char *err, int err_size);

#endif /* AIM_HOST_CALIBFILE_H_ */
