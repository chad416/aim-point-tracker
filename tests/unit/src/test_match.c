/*
 * SPEC F07-F10, §6.5, P04: shot frame on lane time, slots, settling, HIT
 * results (OK, NO_SPOT, OFF_SCREEN, SATURATED, DROPPED, AMBIGUOUS), two
 * weapons in one frame, stray spots, late SHOT_EVENTs, zeroing.
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "aimtrk/match.h"
#include "aimtrk/proto.h"

ZTEST_SUITE(match, NULL, NULL, NULL, NULL, NULL);

static at_match m;
static at_mevent ev[32];

static void setup(int n_weapons)
{
	at_match_cfg c;

	at_match_cfg_default(&c);
	c.n_weapons = n_weapons;
	c.weapon[0] = (at_weapon_cfg){.node = 1, .slot = 0};
	c.weapon[1] = (at_weapon_cfg){.node = 2, .slot = 1};
	zassert_ok(at_match_init(&m, &c));
}

static at_mspot spot(float x, float y)
{
	return (at_mspot){.x_mm = x, .y_mm = y, .snr = 100.0F, .flags = 0U};
}

/* Events of exposure k once it is settled. */
static int settle(int64_t k)
{
	return at_match_poll(&m, at_match_exposure_end(k) + m.cfg.settle_us, ev, 32);
}

/* A break time whose shot pulse falls into exposure k. */
static int64_t break_for(int64_t k)
{
	return (k * AT_FRAME_US) + 1000;
}

ZTEST(match, test_config)
{
	at_match_cfg c;

	at_match_cfg_default(&c);
	c.n_weapons = 2;
	c.weapon[0] = (at_weapon_cfg){1, 0};
	c.weapon[1] = (at_weapon_cfg){2, 1};
	zassert_ok(at_match_init(&m, &c));
	c.n_slots = 1;
	zassert_equal(at_match_init(&m, &c), -1, "at least two slots (SPEC 6.5)");
	c.n_slots = 2;
	c.weapon[1].slot = 0;
	zassert_equal(at_match_init(&m, &c), -1, "shared slot");
	c.weapon[1] = (at_weapon_cfg){1, 1};
	zassert_equal(at_match_init(&m, &c), -1, "shared node");
	c.weapon[1] = (at_weapon_cfg){2, 2};
	zassert_equal(at_match_init(&m, &c), -1, "slot out of range");
	c.weapon[1] = (at_weapon_cfg){0, 1};
	zassert_equal(at_match_init(&m, &c), -1, "node 0");
	c.n_weapons = 5;
	zassert_equal(at_match_init(&m, &c), -1);
}

ZTEST(match, test_shot_frame_rule)
{
	/* Pulse of exposure k starts at k*10 ms + 4.75 ms; it must start >= 2 ms after the break. */
	zassert_equal(at_match_shot_frame(0), 0);
	zassert_equal(at_match_shot_frame(2750), 0, "exactly the margin");
	zassert_equal(at_match_shot_frame(2751), 1);
	zassert_equal(at_match_shot_frame(12750), 1);
	zassert_equal(at_match_shot_frame(123456789), 12346);
	zassert_equal(at_match_shot_frame(-7751), -1);
	zassert_equal(at_match_exposure_end(3), 35500);
}

/* One weapon: aim in its slot frames, a shot in the other slot's frame, zeroing applied. */
ZTEST(match, test_aim_and_hit)
{
	setup(1);
	at_zero *z = at_match_zero(&m, 1);

	zassert_not_null(z);
	zassert_is_null(at_match_zero(&m, 9));
	z->valid = true;
	z->off_x = 5.0;
	z->off_y = -5.0;

	for (int64_t k = 100; k < 110; k++) {
		at_mspot s = spot(100.0F + (float)(k - 100), 50.0F);

		at_match_frame(&m, k, ((k % 2) == 0) ? &s : NULL, ((k % 2) == 0) ? 1 : 0);
		int n = settle(k);

		if (k == 100) {
			zassert_equal(n, 0, "the first sighting is only a candidate");
		} else if ((k % 2) == 0) {
			zassert_equal(n, 1, "k %lld", (long long)k);
			zassert_equal(ev[0].kind, AT_EV_AIM);
			zassert_equal(ev[0].node, 1);
			zassert_equal(ev[0].k, k);
			zassert_within(ev[0].x_mm, 95.0F + (float)(k - 100), 1e-4F, "zeroed");
			zassert_within(ev[0].y_mm, 55.0F, 1e-4F);
		} else {
			zassert_equal(n, 0);
		}
	}
	/* A shot breaking in exposure 110's window; the event arrives before frame 111 is settled. */
	zassert_ok(at_match_shot(&m, 1, 42, break_for(111)));
	at_mspot s = spot(111.0F, 50.0F);

	at_match_frame(&m, 110, NULL, 0);
	zassert_equal(settle(110), 0, "aim pulse replaced? no: 110 is the weapon's aim slot, no spot");
	at_match_frame(&m, 111, &s, 1);
	zassert_equal(at_match_poll(&m, at_match_exposure_end(111) + m.cfg.settle_us - 1, ev, 32), 0,
		      "not settled yet");
	zassert_equal(settle(111), 1);
	zassert_equal(ev[0].kind, AT_EV_HIT);
	zassert_equal(ev[0].result, AT_HIT_OK);
	zassert_equal(ev[0].shot_no, 42);
	zassert_equal(ev[0].t_break_us, break_for(111));
	zassert_within(ev[0].x_mm, 106.0F, 1e-4F);
	zassert_within(ev[0].y_mm, 55.0F, 1e-4F);
	zassert_equal(ev[0].quality, 100);
	zassert_equal(m.stats.shots, 1U);

	/* The track followed the aim (1 mm per frame). */
	const at_track *t = at_match_track(&m, 1);

	zassert_not_null(t);
	zassert_within(t->x, 111.0, 0.5);
	zassert_is_null(at_match_track(&m, 9));
}

/* The shot pulse replaces the aim pulse in the weapon's own slot frame: one HIT, no AIM. */
ZTEST(match, test_shot_in_own_slot)
{
	setup(1);
	at_mspot s = spot(0.0F, 0.0F);

	zassert_ok(at_match_shot(&m, 1, 1, break_for(20)));
	at_match_frame(&m, 20, &s, 1);
	zassert_equal(settle(20), 1);
	zassert_equal(ev[0].kind, AT_EV_HIT);
	zassert_equal(ev[0].result, AT_HIT_OK);
}

ZTEST(match, test_no_hit_reasons)
{
	setup(1);
	at_mspot off = spot(1700.0F, 0.0F);
	at_mspot sat = spot(1690.0F, 10.0F); /* near the track the off-screen shot started */

	off.flags = AT_MSPOT_OFF_SCREEN;
	sat.flags = AT_MSPOT_SATURATED;
	sat.snr = 900.0F;

	zassert_ok(at_match_shot(&m, 1, 1, break_for(1)));
	zassert_ok(at_match_shot(&m, 1, 2, break_for(3)));
	zassert_ok(at_match_shot(&m, 1, 3, break_for(5)));
	zassert_ok(at_match_shot(&m, 1, 4, break_for(7)));
	at_match_frame(&m, 0, NULL, 0);
	at_match_frame(&m, 1, NULL, 0);    /* no spot: emitter blocked */
	at_match_frame(&m, 3, &off, 1);    /* frame 2 missing: never received */
	at_match_frame(&m, 5, &sat, 1);    /* frame 4 missing */
	/* frame 7 never arrives */
	at_match_frame(&m, 8, NULL, 0);
	int n = settle(8);
	int got[6] = {0};
	uint8_t q = 0;

	for (int i = 0; i < n; i++) {
		zassert_equal(ev[i].kind, AT_EV_HIT);
		got[ev[i].result]++;
		if (ev[i].result == AT_HIT_SATURATED) {
			q = ev[i].quality;
		}
		if (ev[i].result == AT_HIT_OFF_SCREEN) {
			zassert_within(ev[i].x_mm, 1700.0F, 1e-4F, "position still reported");
		}
	}
	zassert_equal(n, 4);
	zassert_equal(got[AT_HIT_NO_SPOT], 1);
	zassert_equal(got[AT_HIT_OFF_SCREEN], 1);
	zassert_equal(got[AT_HIT_SATURATED], 1);
	zassert_equal(got[AT_HIT_DROPPED], 1);
	zassert_equal(q, 255, "quality saturates");
	zassert_equal(m.stats.dropped, 4U, "frames 2, 4, 6 and 7");
}

/* Two weapons; both shoot into the same frame: each gets the spot near its own track. */
ZTEST(match, test_two_weapons_same_frame)
{
	setup(2);
	for (int64_t k = 0; k < 10; k++) {
		at_mspot s = ((k % 2) == 0) ? spot(-500.0F, 0.0F) : spot(500.0F, 100.0F);

		at_match_frame(&m, k, &s, 1);
		zassert_equal(settle(k), (k < 2) ? 0 : 1, "k %lld", (long long)k);
		if (k >= 2) {
			zassert_equal(ev[0].node, ((k % 2) == 0) ? 1 : 2);
		}
	}
	zassert_ok(at_match_shot(&m, 2, 7, break_for(10)));
	zassert_ok(at_match_shot(&m, 1, 9, break_for(10)));
	at_mspot two[2] = {spot(503.0F, 98.0F), spot(-497.0F, 2.0F)};

	at_match_frame(&m, 10, two, 2);
	zassert_equal(settle(10), 2);
	for (int i = 0; i < 2; i++) {
		zassert_equal(ev[i].kind, AT_EV_HIT);
		zassert_equal(ev[i].result, AT_HIT_OK);
		zassert_within(ev[i].x_mm, (ev[i].node == 1) ? -497.0F : 503.0F, 1e-4F);
	}

	/* Weapon 1 shoots into weapon 2's aim frame: the aim spot and the shot spot are told apart. */
	zassert_ok(at_match_shot(&m, 1, 10, break_for(11)));
	at_mspot mix[2] = {spot(-495.0F, 3.0F), spot(505.0F, 97.0F)};

	at_match_frame(&m, 11, mix, 2);
	zassert_equal(settle(11), 2);
	for (int i = 0; i < 2; i++) {
		if (ev[i].node == 1) {
			zassert_equal(ev[i].kind, AT_EV_HIT);
			zassert_within(ev[i].x_mm, -495.0F, 1e-4F);
		} else {
			zassert_equal(ev[i].kind, AT_EV_AIM);
			zassert_within(ev[i].x_mm, 505.0F, 1e-4F);
		}
	}
}

/* Both weapons aim at nearly the same point and shoot into the same frame: AMBIGUOUS, not a guess. */
ZTEST(match, test_ambiguous)
{
	setup(2);
	for (int64_t k = 0; k < 10; k++) {
		at_mspot s = ((k % 2) == 0) ? spot(0.0F, 0.0F) : spot(6.0F, 0.0F);

		at_match_frame(&m, k, &s, 1);
		(void)settle(k);
	}
	zassert_ok(at_match_shot(&m, 1, 1, break_for(10)));
	zassert_ok(at_match_shot(&m, 2, 1, break_for(10)));
	at_mspot two[2] = {spot(1.0F, 0.0F), spot(5.0F, 0.0F)};

	at_match_frame(&m, 10, two, 2);
	zassert_equal(settle(10), 2);
	zassert_equal(ev[0].result, AT_HIT_AMBIGUOUS);
	zassert_equal(ev[1].result, AT_HIT_AMBIGUOUS);
	zassert_equal(ev[0].quality, 0);
	zassert_equal(m.stats.ambiguous, 2U);
}

/* A spot that belongs to no expected emitter is counted as stray; outside the gate it is not taken. */
ZTEST(match, test_stray_and_gate)
{
	setup(1);
	for (int64_t k = 0; k < 6; k += 2) {
		at_mspot s = spot(0.0F, 0.0F);

		at_match_frame(&m, k, &s, 1);
		(void)settle(k);
		at_match_frame(&m, k + 1, NULL, 0);
		(void)settle(k + 1);
	}
	at_mspot s2[2] = {spot(900.0F, 0.0F), spot(2.0F, 1.0F)};

	at_match_frame(&m, 6, s2, 2);
	zassert_equal(settle(6), 1);
	zassert_within(ev[0].x_mm, 2.0F, 1e-4F);
	zassert_equal(m.stats.stray_spots, 1U);

	/* Only a spot outside the gate: not this weapon's; the aim is dropped, the spot is stray. */
	at_match_frame(&m, 7, NULL, 0);
	(void)settle(7);
	at_mspot far = spot(400.0F, 0.0F);

	at_match_frame(&m, 8, &far, 1);
	zassert_equal(settle(8), 0);
	zassert_equal(m.stats.stray_spots, 2U);
}

/* The SHOT_EVENT comes after its frame was settled: matched against the stored frame. */
ZTEST(match, test_late_shot)
{
	setup(1);
	at_mspot s = spot(30.0F, 40.0F);

	at_match_frame(&m, 0, NULL, 0);
	at_match_frame(&m, 1, &s, 1);
	zassert_equal(settle(1), 0, "no shot known yet: stray");
	zassert_equal(m.stats.stray_spots, 1U);
	zassert_ok(at_match_shot(&m, 1, 5, break_for(1)));
	at_match_frame(&m, 2, NULL, 0);
	zassert_equal(settle(2), 1);
	zassert_equal(ev[0].result, AT_HIT_OK);
	zassert_within(ev[0].x_mm, 30.0F, 1e-4F);
	zassert_equal(m.stats.late_shots, 1U);
	zassert_equal(m.stats.stray_spots, 0U);

	/* Late, and its frame was never received. */
	zassert_ok(at_match_shot(&m, 1, 6, break_for(-1)));
	zassert_equal(settle(3), 1);
	zassert_equal(ev[0].result, AT_HIT_DROPPED);
	zassert_equal(at_match_shot(&m, 9, 1, 0), -1, "unknown node");
	zassert_equal(m.stats.unknown_node, 1U);
}

ZTEST(match, test_gap_and_limits)
{
	setup(1);
	zassert_equal(at_match_poll(&m, 1000000, ev, 32), 0, "nothing before the first frame");

	/* More spots than kept: the strongest AT_MATCH_MAX_SPOTS stay. */
	at_mspot many[12];

	for (int i = 0; i < 12; i++) {
		many[i] = spot((float)(100 * i), 0.0F);
		many[i].snr = (float)(10 + i);
	}
	at_match_frame(&m, 0, many, 12);
	zassert_equal(m.ring[0].n, AT_MATCH_MAX_SPOTS);
	zassert_within(m.ring[0].s[0].snr, 21.0F, 1e-6F, "strongest first");

	/* The next frame is late: it is waited for (up to drop_wait_us), then dropped. */
	(void)at_match_poll(&m, at_match_exposure_end(0) + m.cfg.settle_us, ev, 32);
	zassert_equal(m.next_settle, 1, "frame 0 settled");
	zassert_equal(at_match_poll(&m, at_match_exposure_end(1) + m.cfg.settle_us, ev, 32), 0);
	zassert_equal(m.next_settle, 1, "frame 1 not received yet: waiting");
	zassert_equal(at_match_poll(&m, at_match_exposure_end(1) + m.cfg.settle_us + m.cfg.drop_wait_us, ev,
				    32),
		      0);
	zassert_equal(m.next_settle, 2, "waited long enough: dropped");

	/* A long gap: the settle point jumps ahead instead of walking every frame. */
	(void)at_match_poll(&m, 3600000000LL, ev, 32);
	zassert_true(m.next_settle > 359000);
	zassert_true(m.stats.dropped > 359000U);

	/* A frame that arrives after its settle time is ignored. */
	at_match_frame(&m, 5, many, 1);
	zassert_equal(m.ring[5].k, 0, "not stored");

	/* The shot table fills up. */
	setup(1);
	for (int i = 0; i < AT_MATCH_SHOTS; i++) {
		zassert_ok(at_match_shot(&m, 1, (uint16_t)i, break_for(1000 + i)));
	}
	zassert_equal(at_match_shot(&m, 1, 99, break_for(2000)), -1);
}

/* A weak spot cannot start a track: interference does not become a weapon's aim. */
ZTEST(match, test_min_init_snr)
{
	setup(1);
	at_mspot weak = spot(800.0F, -400.0F);

	weak.snr = 15.0F;
	at_match_frame(&m, 0, &weak, 1);
	zassert_equal(settle(0), 0, "no AIM from a weak stray");
	zassert_false(m.track[0].active);
	zassert_equal(m.stats.stray_spots, 1U);

	/* The real emitter (strong) is the candidate, even with the weak stray in the frame ... */
	at_mspot both[2] = {weak, spot(100.0F, 50.0F)};

	at_match_frame(&m, 1, NULL, 0);
	(void)settle(1);
	at_match_frame(&m, 2, both, 2);
	zassert_equal(settle(2), 0, "candidate only");
	zassert_true(m.cand[0]);
	/* ... and the weapon's next slot frame confirms it. */
	at_mspot next = spot(101.0F, 50.0F);

	at_match_frame(&m, 3, NULL, 0);
	(void)settle(3);
	at_match_frame(&m, 4, &next, 1);
	zassert_equal(settle(4), 1);
	zassert_within(ev[0].x_mm, 101.0F, 1e-4F);
	zassert_true(m.track[0].active);
	zassert_equal(m.stats.confirmed, 1U);
}

/* A stray seen once is never confirmed: no track, no AIM; a shot then takes only the candidate's spot. */
ZTEST(match, test_unconfirmed_stray)
{
	setup(1);
	at_mspot stray = spot(-300.0F, 200.0F);

	at_match_frame(&m, 0, &stray, 1);
	zassert_equal(settle(0), 0);
	at_match_frame(&m, 1, NULL, 0);
	(void)settle(1);
	/* The stray is gone; the weapon's real spot is far away: a new candidate, no AIM. */
	at_mspot real = spot(500.0F, -100.0F);

	at_match_frame(&m, 2, &real, 1);
	zassert_equal(settle(2), 0);
	zassert_within(m.cand_x[0], 500.0F, 1e-4F);
	/* A shot (hidden) with a stray far from the candidate: NO_SPOT, not the stray. */
	zassert_ok(at_match_shot(&m, 1, 1, break_for(3)));
	at_match_frame(&m, 3, &stray, 1);
	zassert_equal(settle(3), 1);
	zassert_equal(ev[0].result, AT_HIT_NO_SPOT);
	zassert_false(m.track[0].active);

	/* The candidate expires if it is not seen again within two slot cycles. */
	for (int64_t k = 4; k < 9; k++) {
		at_match_frame(&m, k, NULL, 0);
		(void)settle(k);
	}
	at_mspot again = spot(500.0F, -100.0F);

	at_match_frame(&m, 9, NULL, 0);
	(void)settle(9);
	at_match_frame(&m, 10, &again, 1);
	zassert_equal(settle(10), 0, "the old candidate expired: this is a new one");
}

/* A track held by the wrong spot lets go: three slot frames in a row with its weapon's spot elsewhere. */
ZTEST(match, test_reacquire)
{
	setup(1);
	for (int64_t k = 0; k < 6; k += 2) {
		at_mspot s = spot(0.0F, 0.0F);

		at_match_frame(&m, k, &s, 1);
		zassert_equal(settle(k), (k == 0) ? 0 : 1, "first sighting: candidate");
		at_match_frame(&m, k + 1, NULL, 0);
		(void)settle(k + 1);
	}
	/* The weapon's spot is now 600 mm away (the track was wrong, or the aim jumped). */
	int aims = 0;

	for (int64_t k = 6; k < 16; k += 2) {
		at_mspot s = spot(600.0F, 0.0F);

		at_match_frame(&m, k, &s, 1);
		int n = settle(k);

		at_match_frame(&m, k + 1, NULL, 0);
		(void)settle(k + 1);
		if (n == 1) {
			zassert_within(ev[0].x_mm, 600.0F, 1e-4F);
			aims++;
		}
		if (k < 12) {
			zassert_equal(n, 0, "frame %lld: still gated out", (long long)k);
		}
	}
	zassert_equal(m.stats.reacquired, 1U);
	zassert_equal(aims, 1, "after three misses: a candidate at frame 12, confirmed with an AIM at 14");
}

/*
 * A shot of a weapon with neither a track nor a candidate (its aim pulses were not seen): only a new,
 * unexplained spot can be its flash. Before, any spot of at least min_init_snr was taken (the strongest).
 */
ZTEST(match, test_untracked_shot)
{
	setup(1);
	at_mspot glint = spot(500.0F, 300.0F); /* a stray in the other slot's frames, stronger than an emitter */

	glint.snr = 300.0F;
	for (int64_t k = 0; k < 5; k++) {
		at_match_frame(&m, k, ((k % 2) == 1) ? &glint : NULL, ((k % 2) == 1) ? 1 : 0);
		zassert_equal(settle(k), 0);
	}
	zassert_false(m.cand[0], "the weapon's own slot frames were empty");

	/* Its flash is hidden; the stray was there two frames before: not taken. */
	zassert_ok(at_match_shot(&m, 1, 1, break_for(5)));
	at_match_frame(&m, 5, &glint, 1);
	zassert_equal(settle(5), 1);
	zassert_equal(ev[0].result, AT_HIT_NO_SPOT, "not the recurring stray");

	/* Its flash is seen, next to the stray: the new spot is taken, not the stronger stray. */
	at_match_frame(&m, 6, NULL, 0);
	(void)settle(6);
	zassert_ok(at_match_shot(&m, 1, 2, break_for(7)));
	at_mspot two[2] = {glint, spot(-200.0F, 0.0F)};

	at_match_frame(&m, 7, two, 2);
	zassert_equal(settle(7), 1);
	zassert_equal(ev[0].result, AT_HIT_OK);
	zassert_within(ev[0].x_mm, -200.0F, 1e-4F);
	zassert_equal(m.stats.untracked_shots, 2U);

	/* Two new spots and no track to choose by: AMBIGUOUS, not a guess. */
	setup(1);
	for (int64_t k = 0; k < 3; k++) {
		at_match_frame(&m, k, NULL, 0);
		(void)settle(k);
	}
	zassert_ok(at_match_shot(&m, 1, 3, break_for(3)));
	at_mspot pair[2] = {spot(300.0F, 0.0F), spot(-300.0F, 0.0F)};

	at_match_frame(&m, 3, pair, 2);
	zassert_equal(settle(3), 1);
	zassert_equal(ev[0].result, AT_HIT_AMBIGUOUS);
	zassert_equal(m.stats.ambiguous, 1U);

	/* The same rule for a SHOT_EVENT that arrives after its frame was settled. */
	at_match_frame(&m, 4, NULL, 0);
	(void)settle(4);
	at_match_frame(&m, 5, &glint, 1);
	(void)settle(5);
	at_match_frame(&m, 6, NULL, 0);
	(void)settle(6);
	at_match_frame(&m, 7, &glint, 1);
	(void)settle(7);
	zassert_ok(at_match_shot(&m, 1, 4, break_for(7)));
	at_match_frame(&m, 8, NULL, 0);
	zassert_equal(settle(8), 1);
	zassert_equal(ev[0].result, AT_HIT_NO_SPOT, "late, and the stray recurs");
	zassert_equal(m.stats.late_shots, 1U);
}

/* A full pending-shot table loses the shot: counted, so the host can report it. */
ZTEST(match, test_shot_table_full_counted)
{
	setup(1);
	for (int i = 0; i < AT_MATCH_SHOTS; i++) {
		zassert_ok(at_match_shot(&m, 1, (uint16_t)i, break_for(1000 + i)));
	}
	zassert_equal(at_match_shot(&m, 1, 99, break_for(2000)), -1);
	zassert_equal(m.stats.shots_lost, 1U);
	zassert_equal(at_match_shot(&m, 9, 1, break_for(2000)), -1);
	zassert_equal(m.stats.shots_lost, 1U, "an unknown node is counted apart");
	zassert_equal(m.stats.unknown_node, 1U);
}
