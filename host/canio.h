/*
 * CAN transport for aim_host: SocketCAN (live), or log files in candump's -L
 * format for repeatable runs (CI, tests):
 *     (1700000000.123456) can0 501#D204B18C6C0E578F
 * The timestamp is the tracker's own clock at reception (input) or sending
 * (output), in seconds.
 */
#ifndef AIM_HOST_CANIO_H_
#define AIM_HOST_CANIO_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
	int64_t t_local_us;
	uint16_t id;
	uint8_t len;
	uint8_t data[8];
} can_msg;

typedef struct {
	bool socket_mode;
	int sock;
	FILE *in;  /* file mode input (NULL: none) */
	FILE *out; /* sent frames (file mode output, or a copy of what goes on the bus) */
	bool have_next;
	can_msg next;
	uint32_t bad_lines;
	char ifname[32];
} canio;

/* File mode: frames from in_path (may be NULL), sent frames to out_path. */
int canio_open_files(canio *c, const char *in_path, const char *out_path);

/* SocketCAN on ifname; sent frames are also logged to log_path if it is not NULL. */
int canio_open_socket(canio *c, const char *ifname, const char *log_path);

/* File mode: the next input frame without taking it. False at the end. */
bool canio_peek(canio *c, can_msg *m);
void canio_pop(canio *c);

/* Socket mode: read one frame if one is waiting (non-blocking), stamped with now_us. */
bool canio_recv(canio *c, int64_t now_us, can_msg *m);

int canio_send(canio *c, const can_msg *m);
void canio_close(canio *c);

/* Parse / format one log line. */
bool canio_parse_line(const char *line, can_msg *m);
void canio_format_line(const can_msg *m, const char *ifname, char *buf, int size);

#endif /* AIM_HOST_CANIO_H_ */
