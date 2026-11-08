/*
 * Frame source for aim_host (SPEC F01): a stream of frames from the camera
 * simulator (pipe) or a recorded file. Each frame is a 32-byte header and
 * width * height bytes, row-major:
 *
 *   magic "AIMF" | width u16 | height u16 | index u32 (camera frame counter) |
 *   t_local_us i64 (exposure start, the tracker's clock) | exposure_us u32 |
 *   flags u32 | reserved u32                                  (little-endian)
 *
 * The camera is triggered by the lane, so its exposures sit on the lane's
 * 10 ms grid; aim_host turns t_local_us into lane time (timesync) to find the
 * exposure number. A jump in index is a dropped frame.
 */
#ifndef AIM_HOST_FRAMES_H_
#define AIM_HOST_FRAMES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define FRAME_HEADER_SIZE 32

typedef struct {
	uint16_t width, height;
	uint32_t index;
	int64_t t_local_us;
	uint32_t exposure_us;
	uint32_t flags;
} frame_hdr;

typedef struct {
	FILE *f;
	uint8_t *pixels; /* allocated once at open */
	size_t capacity;
	bool have;       /* a frame is buffered (hdr, pixels) */
	frame_hdr hdr;
	uint32_t frames;
} frame_src;

/* path "-" is stdin. capacity: largest frame in pixels. */
int frames_open(frame_src *s, const char *path, size_t capacity);

/* Read the next frame into the buffer. 1: a frame, 0: end of stream, -1: error. */
int frames_next(frame_src *s);

void frames_close(frame_src *s);

#endif /* AIM_HOST_FRAMES_H_ */
