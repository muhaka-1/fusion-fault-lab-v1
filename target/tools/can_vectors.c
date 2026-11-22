/* Prints one encoded, E2E-protected reference frame per message: name, id, 8 bytes.
 * tests/can_vectors.txt is this output, committed. CI checks that C still prints exactly that,
 * and tests/test_can_contract.py checks that Python produces and reads the same bytes.
 *   ./target/build/can_vectors | diff - tests/can_vectors.txt */
#include "can_msg.h"
#include "e2e.h"
#include <stdio.h>

static void out(const char *name, can_frame_t *f, uint8_t ctr)
{
    e2e_tx_t tx = { ctr };
    e2e_protect(&tx, (uint16_t)f->id, f->data);
    printf("%s %03x", name, (unsigned)f->id);
    for (int i = 0; i < 8; ++i) printf(" %02x", f->data[i]);
    printf("\n");
}

int main(void)
{
    can_frame_t f;
    msg_imu_t imu = { -0.0123456, 1.23456 }; msg_encode_imu(&f, &imu); out("imu", &f, 3);
    msg_wheel_t w = { 12.345 }; msg_encode_wheel(&f, &w); out("wheel", &f, 14);
    msg_gnss_t g = { true, -123.456, 4567.891 }; msg_encode_gnss(&f, &g); out("gnss", &f, 0);
    msg_gnss_t gn = { false, 0, 0 }; msg_encode_gnss(&f, &gn); out("gnss_nofix", &f, 1);
    msg_engine_t e = { true, true, 3456, 123.45 }; msg_encode_engine(&f, &e); out("engine", &f, 7);
    msg_inputs_t in = { IN_STALK_LEFT | IN_DIP_SWITCH | IN_FLASH }; msg_encode_inputs(&f, &in); out("inputs", &f, 9);
    msg_nav_state_t s = { NAV_DEGRADED, NAVF_GNSS_ALARM | NAVF_RECOVERY, 271.5, 1.25, 2.4 }; msg_encode_nav_state(&f, &s); out("nav_state", &f, 5);
    msg_nav_pose_t p = { 1000.5, -20.25 }; msg_encode_nav_pose(&f, &p); out("nav_pose", &f, 6);
    msg_nav_diag_t d = { 11.5, 0.00412, 37, 1 }; msg_encode_nav_diag(&f, &d); out("nav_diag", &f, 2);
    msg_nav_cmd_t c = { true, false, 50, 0.999 }; msg_encode_nav_cmd(&f, &c); out("nav_cmd", &f, 4);
    msg_cluster_a_t a = { 1234, CAN_U16_INVALID, LAMP_LEFT | LAMP_POS_AMBER, LAMP2_HAZARD }; msg_encode_cluster_a(&f, &a); out("cluster_a", &f, 8);
    msg_cluster_b_t b = { 27150, POS_TXT_DEGRADED, 125 }; msg_encode_cluster_b(&f, &b); out("cluster_b", &f, 10);
    return 0;
}
