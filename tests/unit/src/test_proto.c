/*
 * SPEC §7: every frame against protocol/tracker.dbc (bytes from cantools,
 * proto_vectors.h), and the saturation of positions and latency.
 */
#include <zephyr/ztest.h>

#include "aimtrk/proto.h"
#include "proto_vectors.h"

ZTEST_SUITE(proto, NULL, NULL, NULL, NULL, NULL);

ZTEST(proto, test_ids)
{
	zassert_equal(AT_CAN_ID(AT_FN_HIT, 1), 0x501);
	zassert_equal(AT_CAN_ID(AT_FN_AIM, 2), 0x582);
	zassert_equal(AT_CAN_ID(AT_FN_TRK_STATUS, AT_TRACKER_NODE), 0x67E);
	zassert_equal(AT_CAN_FN(0x101), AT_FN_SHOT_EVENT);
	zassert_equal(AT_CAN_NODE(0x183), 3);
}

ZTEST(proto, test_hit_against_dbc)
{
	uint8_t d[8];
	at_hit_msg ok = {.shot_no = 1234, .x_mm = 812.25F, .y_mm = -403.5F, .result = AT_HIT_OK,
			 .quality = 87, .latency_ms = 14.3F};
	at_hit_msg edge = {.shot_no = 65535, .x_mm = -2048.0F, .y_mm = 2047.75F,
			   .result = AT_HIT_OFF_SCREEN, .quality = 255, .latency_ms = 25.5F};
	at_hit_msg drop = {.shot_no = 7, .result = AT_HIT_DROPPED, .latency_ms = 0.1F};

	zassert_equal(at_proto_hit(&ok, d), 8);
	zassert_mem_equal(d, VEC_HIT_OK, 8);
	zassert_equal(at_proto_hit(&edge, d), 8);
	zassert_mem_equal(d, VEC_HIT_NEG_EDGE, 8);
	zassert_equal(at_proto_hit(&drop, d), 8);
	zassert_mem_equal(d, VEC_HIT_DROPPED, 8);

	/* Beyond the range: saturates to the edge frame's values. */
	at_hit_msg far = edge;

	far.x_mm = -5000.0F;
	far.y_mm = 9000.0F;
	far.latency_ms = 80.0F;
	zassert_equal(at_proto_hit(&far, d), 8);
	zassert_mem_equal(d, VEC_HIT_NEG_EDGE, 8);

	/* Round trip through the decoder. */
	at_hit_msg back;

	zassert_false(at_proto_hit_decode(d, 7, &back));
	zassert_true(at_proto_hit_decode(VEC_HIT_OK, 8, &back));
	zassert_equal(back.shot_no, 1234);
	zassert_within(back.x_mm, 812.25F, 1e-6F);
	zassert_within(back.y_mm, -403.5F, 1e-6F);
	zassert_equal(back.result, AT_HIT_OK);
	zassert_equal(back.quality, 87);
	zassert_within(back.latency_ms, 14.3F, 1e-5F);
}

ZTEST(proto, test_aim_and_status_against_dbc)
{
	uint8_t d[8];
	at_aim_msg aim = {.frame_seq = 40000, .x_mm = -1499.75F, .y_mm = 844.5F};
	at_status_msg st = {.fps = 99.9F, .dropped = 3, .worst_ms = 1.9F, .ambient = 16, .cal_age_h = 5,
			    .faults = AT_FAULT_FRAMES_DROPPED | AT_FAULT_NO_SYNC};

	zassert_equal(at_proto_aim(&aim, d), 6);
	zassert_mem_equal(d, VEC_AIM, 6);
	zassert_equal(at_proto_status(&st, d), 7);
	zassert_mem_equal(d, VEC_STATUS, 7);
}

ZTEST(proto, test_decode_s1_frames)
{
	uint32_t t;
	at_shot_event se;
	at_shot_detail sd;

	zassert_true(at_proto_sync(VEC_SYNC, 5, &t));
	zassert_equal(t, 4000000123U);
	zassert_false(at_proto_sync(VEC_SYNC, 8, &t));

	zassert_true(at_proto_shot_event(VEC_SHOT_EVENT, 8, &se));
	zassert_equal(se.shot_no, 513);
	zassert_equal(se.t_us, 3000000007U);
	zassert_equal(se.mode, 2);
	zassert_true(se.chambered);
	zassert_equal(se.rounds_left, 29);
	zassert_false(at_proto_shot_event(VEC_SHOT_EVENT, 5, &se));

	zassert_true(at_proto_shot_detail(VEC_SHOT_DETAIL, 8, &sd));
	zassert_equal(sd.shot_no, 513);
	zassert_equal(sd.takeup_ms, 120);
	zassert_equal(sd.squeeze_ms, 1020);
	zassert_equal(sd.jerk, 17);
	zassert_equal(sd.muzzle_rate, 9);
	zassert_equal(sd.cant, -12);
	zassert_equal(sd.flinch_x10, -13);
	zassert_false(at_proto_shot_detail(VEC_SHOT_DETAIL, 6, &sd));
}
