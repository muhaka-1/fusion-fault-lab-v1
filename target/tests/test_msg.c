#include "test.h"
#include "can_msg.h"
#include "e2e.h"
#include "canbus.h"
#include <string.h>

static void imu_roundtrip_and_resolution(void)
{
    can_frame_t f;
    msg_imu_t m = { -0.1234567, 3.21987 }, d;
    msg_encode_imu(&f, &m);
    msg_decode_imu(&f, &d);
    CHECK_EQ(f.id, CAN_ID_IMU);
    CHECK_EQ(f.dlc, 8);
    CHECK_NEAR(d.gyro_radps, m.gyro_radps, 0.5e-6);
    CHECK_NEAR(d.acc_mps2, m.acc_mps2, 0.5e-5);
}

static void imu_saturates_instead_of_wrapping(void)
{
    can_frame_t f;
    msg_imu_t m = { 100.0, -1000.0 }, d;
    msg_encode_imu(&f, &m);
    msg_decode_imu(&f, &d);
    CHECK_NEAR(d.gyro_radps, 8.388607, 1e-6);
    CHECK_NEAR(d.acc_mps2, -83.88608, 1e-5);
}

static void wheel_and_engine_roundtrip(void)
{
    can_frame_t f;
    msg_wheel_t w = { 13.37 }, wd;
    msg_encode_wheel(&f, &w); msg_decode_wheel(&f, &wd);
    CHECK_NEAR(wd.speed_mps, 13.37, 0.005);
    msg_engine_t e = { true, true, 2345.0, 87.65 }, ed;
    msg_encode_engine(&f, &e); msg_decode_engine(&f, &ed);
    CHECK(ed.mil && ed.running);
    CHECK_NEAR(ed.rpm, 2345.0, 0.5);
    CHECK_NEAR(ed.speed_kph, 87.65, 0.005);
    msg_wheel_t neg = { -3.0 };
    msg_encode_wheel(&f, &neg); msg_decode_wheel(&f, &wd);
    CHECK_NEAR(wd.speed_mps, 0.0, 1e-12);
}

static void gnss_fix_flag_and_negative_coordinates(void)
{
    can_frame_t f;
    msg_gnss_t g = { true, -1234.56, 78901.23 }, d;
    msg_encode_gnss(&f, &g); msg_decode_gnss(&f, &d);
    CHECK(d.fix);
    CHECK_NEAR(d.x_m, -1234.56, 0.005);
    CHECK_NEAR(d.y_m, 78901.23, 0.005);
    msg_gnss_t nofix = { false, 5, 5 };
    msg_encode_gnss(&f, &nofix); msg_decode_gnss(&f, &d);
    CHECK(!d.fix);
}

static void nav_state_heading_wraps(void)
{
    can_frame_t f;
    msg_nav_state_t s = { NAV_DEGRADED, NAVF_GNSS_ALARM, -10.0, 1.234, 3.3 }, d;
    msg_encode_nav_state(&f, &s); msg_decode_nav_state(&f, &d);
    CHECK_EQ(d.status, NAV_DEGRADED);
    CHECK_EQ(d.flags, NAVF_GNSS_ALARM);
    CHECK_NEAR(d.heading_deg, 350.0, 0.005);
    CHECK_NEAR(d.pos_std_m, 1.23, 0.006);
    CHECK_NEAR(d.gnss_age_s, 3.3, 0.05);
    s.heading_deg = 359.999;
    msg_encode_nav_state(&f, &s); msg_decode_nav_state(&f, &d);
    CHECK_NEAR(d.heading_deg, 0.0, 1e-9);
}

static void nav_cmd_and_diag_roundtrip(void)
{
    can_frame_t f;
    msg_nav_cmd_t c = { false, true, 0, 0.999 }, cd;
    msg_encode_nav_cmd(&f, &c); msg_decode_nav_cmd(&f, &cd);
    CHECK(!cd.gating && cd.realign);
    CHECK_EQ(cd.lockout_rejects, 0);
    CHECK_NEAR(cd.gate_prob, 0.999, 1e-9);
    msg_nav_diag_t g = { 12.5, -0.0042, 300, 2 }, gd;
    msg_encode_nav_diag(&f, &g); msg_decode_nav_diag(&f, &gd);
    CHECK_NEAR(gd.speed_mps, 12.5, 0.005);
    CHECK_NEAR(gd.bias_radps, -0.0042, 0.5e-5);
    CHECK_EQ(gd.gnss_rejects, 300 & 0xFF);
    CHECK_EQ(gd.recoveries, 2);
}

static void cluster_frames_roundtrip(void)
{
    can_frame_t f;
    msg_cluster_a_t a = { CAN_U16_INVALID, 3100, LAMP_LEFT | LAMP_ECU_FAULT, LAMP2_HAZARD }, ad;
    msg_encode_cluster_a(&f, &a); msg_decode_cluster_a(&f, &ad);
    CHECK(memcmp(&a, &ad, sizeof a) == 0);
    msg_cluster_b_t b = { 12345, POS_TXT_ALIGNING, 77 }, bd;
    msg_encode_cluster_b(&f, &b); msg_decode_cluster_b(&f, &bd);
    CHECK_EQ(bd.heading_cdeg, 12345);
    CHECK_EQ(bd.text, POS_TXT_ALIGNING);
    CHECK_EQ(bd.pos_std_cm, 77);
}

static void every_message_has_its_id_and_8_bytes(void)
{
    can_frame_t f;
    msg_imu_t i = { 0 }; msg_encode_imu(&f, &i);               CHECK_EQ(f.id, 0x100); CHECK_EQ(f.dlc, 8);
    msg_wheel_t w = { 0 }; msg_encode_wheel(&f, &w);           CHECK_EQ(f.id, 0x110);
    msg_gnss_t g = { 0 }; msg_encode_gnss(&f, &g);             CHECK_EQ(f.id, 0x120);
    msg_engine_t e = { 0 }; msg_encode_engine(&f, &e);         CHECK_EQ(f.id, 0x200);
    msg_inputs_t n = { 0 }; msg_encode_inputs(&f, &n);         CHECK_EQ(f.id, 0x210);
    msg_nav_state_t s = { 0 }; msg_encode_nav_state(&f, &s);   CHECK_EQ(f.id, 0x300);
    msg_cluster_a_t a = { 0 }; msg_encode_cluster_a(&f, &a);   CHECK_EQ(f.id, 0x400);
    msg_cluster_b_t b = { 0 }; msg_encode_cluster_b(&f, &b);   CHECK_EQ(f.id, 0x401);
    CHECK_EQ(f.dlc, 8);
}

static void encode_protect_check_decode(void)
{
    /* the full sender -> receiver path: encode, E2E-protect, check, decode */
    e2e_tx_t tx = { 0 };
    e2e_rx_t rx;
    e2e_rx_init(&rx, 2, 0);
    can_frame_t f;
    msg_engine_t e = { false, true, 1800.0, 63.2 }, d;
    msg_encode_engine(&f, &e);
    e2e_protect(&tx, (uint16_t)f.id, f.data);
    CHECK_EQ(e2e_check(&rx, (uint16_t)f.id, f.data), E2E_INITIAL);
    msg_decode_engine(&f, &d);
    CHECK(d.running && !d.mil);                 /* the counter nibble did not disturb the flags */
    CHECK_NEAR(d.rpm, 1800.0, 0.5);
    f.data[4] ^= 0x01;                          /* one bit of the speed flips on the wire */
    CHECK_EQ(e2e_check(&rx, (uint16_t)f.id, f.data), E2E_BAD_CRC);
}

static void rounding_is_half_away_from_zero(void)
{
    /* lround(): 0.125 m -> 12.5 cm -> 13; -0.125 m -> -13. Python must do the same. */
    can_frame_t f;
    msg_nav_pose_t p = { 0.125, -0.125 }, d;
    msg_encode_nav_pose(&f, &p); msg_decode_nav_pose(&f, &d);
    CHECK_NEAR(d.x_m, 0.13, 1e-12);
    CHECK_NEAR(d.y_m, -0.13, 1e-12);
}

static void wire_format_roundtrip_and_rejects(void)
{
    /* M11: one CAN frame = one 16-byte UDP datagram. Anything else is dropped. */
    can_frame_t f = { 0x401, 8, { 1, 2, 3, 4, 5, 6, 7, 8 } }, g;
    unsigned char w[CANBUS_WIRE_LEN];
    canbus_pack(&f, w);
    CHECK_EQ(w[0], 0x01); CHECK_EQ(w[1], 0x04); CHECK_EQ(w[4], 8);   /* id little endian, dlc */
    CHECK_EQ(canbus_unpack(w, CANBUS_WIRE_LEN, &g), 0);
    CHECK(g.id == f.id && g.dlc == 8 && memcmp(g.data, f.data, 8) == 0);
    CHECK_EQ(canbus_unpack(w, 15, &g), -1);                  /* short datagram */
    w[4] = 9;
    CHECK_EQ(canbus_unpack(w, CANBUS_WIRE_LEN, &g), -1);     /* dlc > 8 */
    w[4] = 8; w[1] = 0x08;
    CHECK_EQ(canbus_unpack(w, CANBUS_WIRE_LEN, &g), -1);     /* id > 11 bit */
}

void suite_msg(void)
{
    RUN(imu_roundtrip_and_resolution);
    RUN(imu_saturates_instead_of_wrapping);
    RUN(wheel_and_engine_roundtrip);
    RUN(gnss_fix_flag_and_negative_coordinates);
    RUN(nav_state_heading_wraps);
    RUN(nav_cmd_and_diag_roundtrip);
    RUN(cluster_frames_roundtrip);
    RUN(every_message_has_its_id_and_8_bytes);
    RUN(encode_protect_check_decode);
    RUN(rounding_is_half_away_from_zero);
    RUN(wire_format_roundtrip_and_rejects);
}
