/*
 * Read calib.json, see calibfile.h.
 */
#include "calibfile.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CALIB_VERSION 2

uint32_t crc32_le(uint32_t crc, const uint8_t *p, size_t n)
{
	crc = ~crc;
	for (size_t i = 0; i < n; i++) {
		crc ^= p[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
		}
	}
	return ~crc;
}

/* Position just after `"key":` inside [s, end), or NULL. */
static const char *after_key(const char *s, const char *end, const char *key)
{
	char pat[64];
	size_t n = (size_t)snprintf(pat, sizeof(pat), "\"%s\"", key);

	for (const char *p = s; (p + n) <= end; p++) {
		if (memcmp(p, pat, n) != 0) {
			continue;
		}
		const char *q = p + n;

		while ((q < end) && ((*q == ' ') || (*q == '\n') || (*q == '\r') || (*q == '\t'))) {
			q++;
		}
		if ((q < end) && (*q == ':')) {
			return q + 1;
		}
	}
	return NULL;
}

/* The object or array starting at the first `open` at or after p: [*begin, *stop). */
static int span(const char *p, const char *end, char open, char close, const char **begin, const char **stop)
{
	while ((p < end) && (*p != open)) {
		p++;
	}
	if (p >= end) {
		return -1;
	}
	int depth = 0;

	for (const char *q = p; q < end; q++) {
		depth += (*q == open) ? 1 : ((*q == close) ? -1 : 0);
		if (depth == 0) {
			*begin = p;
			*stop = q + 1;
			return 0;
		}
	}
	return -1;
}

static int number(const char *s, const char *end, const char *key, double *v)
{
	const char *p = after_key(s, end, key);
	char *e;

	if (p == NULL) {
		return -1;
	}
	*v = strtod(p, &e);
	return (e == p) ? -1 : 0;
}

int calib_load(const char *path, at_geom *g, int64_t *created_unix, char *err, int err_size)
{
	FILE *f = fopen(path, "rb");
	char *buf = NULL;
	int rc = -1;

	if (f == NULL) {
		snprintf(err, (size_t)err_size, "cannot open %s", path);
		return -1;
	}
	(void)fseek(f, 0, SEEK_END);
	long size = ftell(f);

	(void)fseek(f, 0, SEEK_SET);
	if ((size <= 0) || (size > (1L << 22)) || ((buf = malloc((size_t)size + 1U)) == NULL) ||
	    (fread(buf, 1, (size_t)size, f) != (size_t)size)) {
		snprintf(err, (size_t)err_size, "cannot read %s", path);
		goto out;
	}
	buf[size] = '\0';
	const char *end = buf + size;
	double version = 0.0, created = 0.0, crc = -1.0;

	if ((number(buf, end, "version", &version) != 0) || ((int)version != CALIB_VERSION)) {
		snprintf(err, (size_t)err_size, "calib.json version %g, expected %d", version, CALIB_VERSION);
		goto out;
	}
	(void)number(buf, end, "created_unix", &created);
	*created_unix = (int64_t)created;

	const char *gb, *ge, *lb, *le, *hb, *he;
	const char *gk = after_key(buf, end, "geometry");

	if ((gk == NULL) || (span(gk, end, '{', '}', &gb, &ge) != 0)) {
		snprintf(err, (size_t)err_size, "no geometry");
		goto out;
	}
	const char *lk = after_key(gb, ge, "lens");
	const char *hk = after_key(gb, ge, "to_cam");

	if ((lk == NULL) || (hk == NULL) || (span(lk, ge, '{', '}', &lb, &le) != 0) ||
	    (span(hk, ge, '[', ']', &hb, &he) != 0)) {
		snprintf(err, (size_t)err_size, "geometry incomplete");
		goto out;
	}
	double v[18];
	static const char *const lens_keys[7] = {"f", "cx", "cy", "k1", "k2", "p1", "p2"};

	for (int i = 0; i < 7; i++) {
		if (number(lb, le, lens_keys[i], &v[i]) != 0) {
			snprintf(err, (size_t)err_size, "lens.%s missing", lens_keys[i]);
			goto out;
		}
	}
	const char *p = hb + 1;

	for (int i = 0; i < 9; i++) {
		char *e;

		v[7 + i] = strtod(p, &e);
		if ((e == p) || (e >= he)) {
			snprintf(err, (size_t)err_size, "to_cam needs 9 numbers");
			goto out;
		}
		p = e;
		while ((p < he) && ((*p == ',') || (*p == ' ') || (*p == '\n'))) {
			p++;
		}
	}
	if ((number(gb, ge, "screen_w", &v[16]) != 0) || (number(gb, ge, "screen_h", &v[17]) != 0) ||
	    (number(buf, end, "geometry_crc32", &crc) != 0)) {
		snprintf(err, (size_t)err_size, "screen size or geometry_crc32 missing");
		goto out;
	}
	uint8_t bytes[18 * 8];

	for (int i = 0; i < 18; i++) {
		uint64_t bits;

		memcpy(&bits, &v[i], 8);
		for (int b = 0; b < 8; b++) {
			bytes[(8 * i) + b] = (uint8_t)(bits >> (8 * b));
		}
	}
	if (crc32_le(0U, bytes, sizeof(bytes)) != (uint32_t)crc) {
		snprintf(err, (size_t)err_size, "geometry_crc32 mismatch");
		goto out;
	}
	at_lens lens = {v[0], v[1], v[2], v[3], v[4], v[5], v[6]};
	at_h33 h;

	memcpy(h.m, &v[7], sizeof(h.m));
	if (at_geom_init(g, &lens, &h, v[16], v[17]) != 0) {
		snprintf(err, (size_t)err_size, "geometry not usable");
		goto out;
	}
	rc = 0;
out:
	free(buf);
	fclose(f);
	return rc;
}
