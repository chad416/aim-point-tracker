/*
 * Which weapon made which spot (SPEC F07-F10, §6.5).
 *
 * Exposure k starts at lane time k * AT_FRAME_US + AT_EXPOSURE_START_US and
 * lasts AT_EXPOSURE_US. Weapon slot s flashes an aim pulse in the middle of
 * exposure k when k mod n_slots = s. At a trigger break at lane time t the
 * shooter flashes its shot pulse in the first exposure whose pulse starts at
 * least AT_STROBE_MARGIN_US after t (at_match_shot_frame); in that frame the
 * shot pulse is the shooter's only pulse. The weapon firmware (v0.6 on) uses
 * the same rule.
 *
 * The caller feeds the spots of every processed frame (mapped to the screen)
 * and every SHOT_EVENT. A frame is settled settle_us after its exposure ends,
 * when its SHOT_EVENTs have arrived. A frame that has not been processed by
 * then is waited for, until a newer frame arrives (frames come in order, so
 * it was dropped) or drop_wait_us has passed. Then the expected emitters of the frame
 * (aim pulses of its slot, shot pulses due in it) are assigned to its spots
 * by distance to each weapon's predicted aim point, inside a gate. A shot
 * always gives a HIT event (with a no-hit reason if needed); an aim pulse
 * with a spot gives an AIM event. Two assignments whose costs are closer than
 * ambiguity_mm are ambiguous: their shots are AMBIGUOUS, their aim is dropped.
 *
 * A weapon without a track takes only a spot of at least min_init_snr (weak
 * interference cannot start a track), and that spot is only a candidate:
 * the track is confirmed when the weapon's next slot frame (or shot) has a
 * spot inside the gate around it, as a real emitter does in every slot
 * frame and a stray on a moving edge does not. Until then a shot takes only
 * a spot consistent with the candidate. A shot of a weapon with neither a
 * track nor a candidate (its aim pulses were not seen) has no position to
 * compare with: it takes a spot only if exactly one strong spot in its frame
 * is unexplained, that is not near another emitter's prediction and not
 * already there in either of the two frames before (a glint, a hot pixel, a
 * recurring stray). Two or more: AMBIGUOUS; none: NO_SPOT. A one-off stray
 * in exactly that frame, while the weapon's own flash is hidden, would still
 * be taken. A track that misses its weapon's
 * spot reacquire_misses slot frames in a row while another spot is left
 * untaken is dropped, so the next slot frame starts it on the strongest spot
 * (IT09: a track held by a recurring stray while the real spot was elsewhere).
 *
 * Tracks run on the raw positions; events carry positions after zeroing.
 * Fixed tables, no heap.
 */
#ifndef AIMTRK_MATCH_H_
#define AIMTRK_MATCH_H_

#include <stdbool.h>
#include <stdint.h>

#include "aimtrk/track.h"
#include "aimtrk/zero.h"

#define AT_FRAME_US          10000
#define AT_EXPOSURE_START_US 4500
#define AT_EXPOSURE_US       1000
#define AT_PULSE_US          500
#define AT_PULSE_START_US    (AT_EXPOSURE_START_US + ((AT_EXPOSURE_US - AT_PULSE_US) / 2)) /* 4750 */
/* Break to shot pulse, at least: the weapon's own fire path takes up to 1.5 ms (its T01). */
#define AT_STROBE_MARGIN_US  2000

#define AT_MAX_WEAPONS     4
#define AT_MATCH_MAX_SPOTS 8
#define AT_MATCH_RING      32  /* frames kept, 320 ms */
#define AT_MATCH_SHOTS     16  /* pending shots */

/* at_mspot.flags */
#define AT_MSPOT_OFF_SCREEN 0x01U
#define AT_MSPOT_SATURATED  0x02U /* saturated and the rim fit failed: position unreliable */

typedef struct {
	float x_mm, y_mm; /* raw (before zeroing) */
	float snr;
	uint8_t flags;
} at_mspot;

typedef struct {
	uint8_t node; /* CAN node of the weapon (1..127) */
	uint8_t slot; /* aim slot, 0..n_slots-1 */
} at_weapon_cfg;

typedef struct {
	int n_slots; /* >= 2 (SPEC §6.5) */
	int n_weapons;
	at_weapon_cfg weapon[AT_MAX_WEAPONS];
	double gate_mm;       /* a spot further than this from a weapon's prediction is not its spot */
	double ambiguity_mm;  /* swapped assignment cheaper than best + this: ambiguous */
	int64_t settle_us;    /* frame settled this long after its exposure ends */
	int64_t drop_wait_us; /* a frame not received by then (and no newer one) is dropped */
	float min_init_snr;   /* a weapon without a track only takes a spot at least this strong */
	int reacquire_misses; /* slot frames in a row with another spot untaken and none for us: drop the track */
	at_track_cfg track;
} at_match_cfg;

enum at_mevent_kind {
	AT_EV_HIT,
	AT_EV_AIM,
};

typedef struct {
	uint8_t kind; /* enum at_mevent_kind */
	uint8_t node;
	int64_t k;            /* exposure */
	uint16_t shot_no;     /* HIT */
	int64_t t_break_us;   /* HIT */
	float x_mm, y_mm;     /* after zeroing (HIT: valid for OK, OFF_SCREEN, SATURATED) */
	uint8_t result;       /* HIT: enum at_hit_result */
	uint8_t quality;      /* HIT: SNR, saturating */
} at_mevent;

typedef struct {
	bool used;
	int64_t k;
	bool present; /* false: not received (dropped) */
	bool settled;
	int n;
	at_mspot s[AT_MATCH_MAX_SPOTS];
	uint8_t taken[AT_MATCH_MAX_SPOTS];
} at_mframe;

typedef struct {
	bool active;
	uint8_t node;
	uint16_t shot_no;
	int64_t t_break_us;
	int64_t k;
} at_mshot;

typedef struct {
	uint32_t frames, dropped, stray_spots, shots, late_shots, ambiguous, unknown_node, reacquired, confirmed;
	uint32_t shots_lost;     /* SHOT_EVENTs refused because the pending-shot table was full: no HIT */
	uint32_t untracked_shots; /* shots of a weapon without a track or candidate (see at_match_shot) */
} at_match_stats;

typedef struct {
	at_match_cfg cfg;
	at_track track[AT_MAX_WEAPONS];
	at_zero zero[AT_MAX_WEAPONS];
	at_mframe ring[AT_MATCH_RING];
	at_mshot shot[AT_MATCH_SHOTS];
	uint8_t miss[AT_MAX_WEAPONS]; /* aim slot frames in a row without our spot, another one untaken */
	bool cand[AT_MAX_WEAPONS];    /* a candidate spot for a new track */
	float cand_x[AT_MAX_WEAPONS], cand_y[AT_MAX_WEAPONS];
	int64_t cand_k[AT_MAX_WEAPONS];
	bool started;
	int64_t next_settle; /* next exposure to settle */
	int64_t newest;      /* newest exposure received */
	at_match_stats stats;
} at_match;

void at_match_cfg_default(at_match_cfg *cfg);

/* Returns 0, or -1 for an invalid configuration (n_slots < 2, slots out of range or shared). */
int at_match_init(at_match *m, const at_match_cfg *cfg);

/* The exposure that carries the shot pulse of a break at lane time t_break_us. */
int64_t at_match_shot_frame(int64_t t_break_us);

/* Lane time at which exposure k ends. */
int64_t at_match_exposure_end(int64_t k);

/* Spots of processed frame k (at most AT_MATCH_MAX_SPOTS are kept, highest SNR first). */
void at_match_frame(at_match *m, int64_t k, const at_mspot *spots, int n);

/*
 * A SHOT_EVENT: weapon node, shot number, break time (unwrapped lane time).
 * -1: unknown node (stats.unknown_node) or the pending-shot table is full
 * (stats.shots_lost); no HIT follows either.
 */
int at_match_shot(at_match *m, uint8_t node, uint16_t shot_no, int64_t t_break_us);

/* Lane time of the middle of the pulses in exposure k (aim and shot pulses alike). */
int64_t at_match_pulse_mid(int64_t k);

/* Settle every frame due by lane time now_us; write up to max events, return their number. */
int at_match_poll(at_match *m, int64_t now_us, at_mevent *out, int max);

/* The weapon's zeroing state (NULL: unknown node). */
at_zero *at_match_zero(at_match *m, uint8_t node);

/* The weapon's track (NULL: unknown node). */
const at_track *at_match_track(const at_match *m, uint8_t node);

#endif /* AIMTRK_MATCH_H_ */
