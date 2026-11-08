/*
 * CAN transport for aim_host: SocketCAN or candump -L log files.
 */
#include "canio.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

/* Static buffer for the sent-frames log, so stdio does not allocate one on first use. */
static char out_buf[1 << 16];

bool canio_parse_line(const char *line, can_msg *m)
{
	long long sec;
	long usec;
	char ifname[32];
	char rest[64];

	if (sscanf(line, " (%lld.%ld) %31s %63s", &sec, &usec, ifname, rest) != 4) {
		return false;
	}
	char *hash = strchr(rest, '#');

	if ((hash == NULL) || (hash == rest)) {
		return false;
	}
	*hash = '\0';
	char *end;
	unsigned long id = strtoul(rest, &end, 16);

	if ((*end != '\0') || (id > 0x7FFUL)) {
		return false;
	}
	const char *hex = hash + 1;
	size_t n = strlen(hex);

	if (((n % 2U) != 0U) || (n > 16U)) {
		return false;
	}
	memset(m, 0, sizeof(*m));
	for (size_t i = 0; i < (n / 2U); i++) {
		char byte[3] = {hex[2U * i], hex[(2U * i) + 1U], '\0'};

		m->data[i] = (uint8_t)strtoul(byte, &end, 16);
		if (*end != '\0') {
			return false;
		}
	}
	m->t_local_us = ((int64_t)sec * 1000000) + usec;
	m->id = (uint16_t)id;
	m->len = (uint8_t)(n / 2U);
	return true;
}

void canio_format_line(const can_msg *m, const char *ifname, char *buf, int size)
{
	int n = snprintf(buf, (size_t)size, "(%" PRId64 ".%06" PRId64 ") %s %03X#", m->t_local_us / 1000000,
			 m->t_local_us % 1000000, ifname, m->id);

	for (int i = 0; (i < m->len) && (n < (size - 3)); i++) {
		n += snprintf(buf + n, (size_t)(size - n), "%02X", m->data[i]);
	}
}

int canio_open_files(canio *c, const char *in_path, const char *out_path)
{
	memset(c, 0, sizeof(*c));
	c->sock = -1;
	snprintf(c->ifname, sizeof(c->ifname), "can0");
	if (in_path != NULL) {
		c->in = fopen(in_path, "r");
		if (c->in == NULL) {
			return -1;
		}
	}
	c->out = fopen(out_path, "w");
	if (c->out == NULL) {
		return -1;
	}
	(void)setvbuf(c->out, out_buf, _IOFBF, sizeof(out_buf));
	return 0;
}

int canio_open_socket(canio *c, const char *ifname, const char *log_path)
{
	struct ifreq ifr;
	struct sockaddr_can addr;

	memset(c, 0, sizeof(*c));
	c->socket_mode = true;
	snprintf(c->ifname, sizeof(c->ifname), "%s", ifname);
	c->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (c->sock < 0) {
		return -1;
	}
	memset(&ifr, 0, sizeof(ifr));
	snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", ifname);
	if (ioctl(c->sock, SIOCGIFINDEX, &ifr) < 0) {
		return -1;
	}
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(c->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		return -1;
	}
	(void)fcntl(c->sock, F_SETFL, O_NONBLOCK);
	if (log_path != NULL) {
		c->out = fopen(log_path, "w");
		if (c->out != NULL) {
			(void)setvbuf(c->out, out_buf, _IOFBF, sizeof(out_buf));
		}
	}
	return 0;
}

bool canio_peek(canio *c, can_msg *m)
{
	char line[256];

	while (!c->have_next && (c->in != NULL) && (fgets(line, sizeof(line), c->in) != NULL)) {
		if ((line[0] == '\n') || (line[0] == '#')) {
			continue;
		}
		if (canio_parse_line(line, &c->next)) {
			c->have_next = true;
		} else {
			c->bad_lines++;
		}
	}
	if (c->have_next) {
		*m = c->next;
	}
	return c->have_next;
}

void canio_pop(canio *c)
{
	c->have_next = false;
}

bool canio_recv(canio *c, int64_t now_us, can_msg *m)
{
	struct can_frame f;
	ssize_t n = read(c->sock, &f, sizeof(f));

	if (n != (ssize_t)sizeof(f)) {
		return false;
	}
	if ((f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U) {
		return false;
	}
	memset(m, 0, sizeof(*m));
	m->t_local_us = now_us;
	m->id = (uint16_t)(f.can_id & CAN_SFF_MASK);
	m->len = (f.len <= 8U) ? f.len : 8U;
	memcpy(m->data, f.data, m->len);
	return true;
}

int canio_send(canio *c, const can_msg *m)
{
	if (c->out != NULL) {
		char line[128];

		canio_format_line(m, c->ifname, line, (int)sizeof(line));
		fprintf(c->out, "%s\n", line);
	}
	if (c->socket_mode) {
		struct can_frame f;

		memset(&f, 0, sizeof(f));
		f.can_id = m->id;
		f.len = m->len;
		memcpy(f.data, m->data, m->len);
		if (write(c->sock, &f, sizeof(f)) != (ssize_t)sizeof(f)) {
			return -1;
		}
	}
	return 0;
}

void canio_close(canio *c)
{
	if (c->in != NULL) {
		fclose(c->in);
	}
	if (c->out != NULL) {
		fclose(c->out);
	}
	if (c->sock >= 0) {
		close(c->sock);
	}
	c->in = NULL;
	c->out = NULL;
	c->sock = -1;
}
