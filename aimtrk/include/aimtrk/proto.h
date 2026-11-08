/*
 * CAN frames of the tracker (SPEC §7, protocol/tracker.dbc).
 *
 * 11-bit ID = (function << 7) | node. The tracker reads the lane's SYNC and
 * the weapons' SHOT_EVENT and SHOT_DETAIL, and sends HIT and AIM (ID carries
 * the shooter's node) and TRK_STATUS (node 0x7E). Little-endian (Intel)
 * signal layout, as the DBC; a unit test checks every encoder against
 * cantools.
 */
#ifndef AIMTRK_PROTO_H_
#define AIMTRK_PROTO_H_

#include <stdbool.h>
#include <stdint.h>

#define AT_FN_SYNC        0x1U
#define AT_FN_SHOT_EVENT  0x2U
#define AT_FN_SHOT_DETAIL 0x3U
#define AT_FN_HIT         0xAU
#define AT_FN_AIM         0xBU
#define AT_FN_TRK_STATUS  0xCU
#define AT_TRACKER_NODE   0x7EU

#define AT_CAN_ID(fn, node) ((uint16_t)(((fn) << 7) | ((node) & 0x7FU)))
#define AT_CAN_FN(id)       ((uint8_t)(((id) >> 7) & 0xFU))
#define AT_CAN_NODE(id)     ((uint8_t)((id) & 0x7FU))

enum at_hit_result {
	AT_HIT_OK = 0,
	AT_HIT_NO_SPOT = 1,
	AT_HIT_OFF_SCREEN = 2,
	AT_HIT_AMBIGUOUS = 3,
	AT_HIT_SATURATED = 4,
	AT_HIT_DROPPED = 5,
};

/* TRK_STATUS faults */
#define AT_FAULT_NO_CALIBRATION  0x01U
#define AT_FAULT_CALIBRATION_OLD 0x02U
#define AT_FAULT_FRAMES_DROPPED  0x04U
#define AT_FAULT_DETECT_OVERFLOW 0x08U
#define AT_FAULT_SLOW            0x10U
#define AT_FAULT_NO_SYNC         0x20U
#define AT_FAULT_CAN_LOSS        0x40U /* a frame could not be sent, or a SHOT_EVENT was not taken */

typedef struct {
	uint16_t shot_no;
	float x_mm, y_mm;     /* 0.25 mm steps, saturating at +-2048 */
	uint8_t result;       /* enum at_hit_result */
	uint8_t quality;      /* peak SNR, saturating */
	float latency_ms;     /* 0.1 ms steps, saturating at 25.5 */
} at_hit_msg;

typedef struct {
	uint16_t frame_seq;
	float x_mm, y_mm;
} at_aim_msg;

typedef struct {
	float fps;
	uint8_t dropped;
	float worst_ms;
	uint8_t ambient;
	uint8_t cal_age_h;
	uint8_t faults;
} at_status_msg;

typedef struct {
	uint16_t shot_no;
	uint32_t t_us; /* lane time of the trigger break */
	uint8_t mode;
	bool chambered;
	uint8_t rounds_left;
} at_shot_event;

typedef struct {
	uint16_t shot_no;
	uint16_t takeup_ms, squeeze_ms;
	uint8_t jerk, muzzle_rate;
	int8_t cant;          /* deg */
	int8_t flinch_x10;    /* 0.1 deg */
} at_shot_detail;

/* Encoders: return the data length (HIT 8, AIM 6, TRK_STATUS 7). */
int at_proto_hit(const at_hit_msg *m, uint8_t out[8]);
int at_proto_aim(const at_aim_msg *m, uint8_t out[8]);
int at_proto_status(const at_status_msg *m, uint8_t out[8]);

/* Decoders: false if the length is wrong. */
bool at_proto_sync(const uint8_t *d, int len, uint32_t *lane_time_us);
bool at_proto_shot_event(const uint8_t *d, int len, at_shot_event *m);
bool at_proto_shot_detail(const uint8_t *d, int len, at_shot_detail *m);
bool at_proto_hit_decode(const uint8_t *d, int len, at_hit_msg *m);

#endif /* AIMTRK_PROTO_H_ */
