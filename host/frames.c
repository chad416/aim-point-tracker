/*
 * Frame source for aim_host.
 */
#include "frames.h"

#include <stdlib.h>
#include <string.h>

static uint16_t u16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int frames_open(frame_src *s, const char *path, size_t capacity)
{
	memset(s, 0, sizeof(*s));
	s->f = (strcmp(path, "-") == 0) ? stdin : fopen(path, "rb");
	if (s->f == NULL) {
		return -1;
	}
	s->pixels = malloc(capacity);
	s->capacity = capacity;
	return (s->pixels == NULL) ? -1 : 0;
}

int frames_next(frame_src *s)
{
	uint8_t h[FRAME_HEADER_SIZE];

	s->have = false;
	size_t got = fread(h, 1, sizeof(h), s->f);

	if (got == 0U) {
		return 0;
	}
	if ((got != sizeof(h)) || (memcmp(h, "AIMF", 4) != 0)) {
		return -1;
	}
	s->hdr.width = u16(&h[4]);
	s->hdr.height = u16(&h[6]);
	s->hdr.index = u32(&h[8]);
	s->hdr.t_local_us = (int64_t)((uint64_t)u32(&h[12]) | ((uint64_t)u32(&h[16]) << 32));
	s->hdr.exposure_us = u32(&h[20]);
	s->hdr.flags = u32(&h[24]);
	size_t n = (size_t)s->hdr.width * s->hdr.height;

	if ((n > s->capacity) || (fread(s->pixels, 1, n, s->f) != n)) {
		return -1;
	}
	s->have = true;
	s->frames++;
	return 1;
}

void frames_close(frame_src *s)
{
	if ((s->f != NULL) && (s->f != stdin)) {
		fclose(s->f);
	}
	free(s->pixels);
	memset(s, 0, sizeof(*s));
}
