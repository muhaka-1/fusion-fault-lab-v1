#include "can_msg.h"
#include <math.h>
#include <string.h>

static void begin(can_frame_t *f, uint32_t id, uint8_t flags)
{
    memset(f, 0, sizeof *f);
    f->id = id;
    f->dlc = 8;
    f->data[1] = (uint8_t)(flags << 4);
}
static uint8_t flags_of(const can_frame_t *f) { return (uint8_t)(f->data[1] >> 4); }

static int32_t sat(double v, double lo, double hi)
{
    if (!(v == v)) return 0;               /* NaN */
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (int32_t)lround(v);
}
static void put_u16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void put_i24(uint8_t *p, int32_t v)
{
    const uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16);
}
static int32_t get_i24(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
    if (u & 0x800000u) u |= 0xFF000000u;
    return (int32_t)u;
}
#define I24_MAX 8388607.0
#define I24_MIN (-8388608.0)

void msg_encode_imu(can_frame_t *f, const msg_imu_t *m)
{
    begin(f, CAN_ID_IMU, 0);
    put_i24(&f->data[2], sat(m->gyro_radps * 1e6, I24_MIN, I24_MAX));
    put_i24(&f->data[5], sat(m->acc_mps2 * 1e5, I24_MIN, I24_MAX));
}
void msg_decode_imu(const can_frame_t *f, msg_imu_t *m)
{
    m->gyro_radps = get_i24(&f->data[2]) * 1e-6;
    m->acc_mps2 = get_i24(&f->data[5]) * 1e-5;
}

void msg_encode_wheel(can_frame_t *f, const msg_wheel_t *m)
{
    begin(f, CAN_ID_WHEEL, 0);
    put_u16(&f->data[2], (uint32_t)sat(m->speed_mps * 100.0, 0, 65534));
}
void msg_decode_wheel(const can_frame_t *f, msg_wheel_t *m) { m->speed_mps = get_u16(&f->data[2]) * 0.01; }

void msg_encode_gnss(can_frame_t *f, const msg_gnss_t *m)
{
    begin(f, CAN_ID_GNSS, m->fix ? 1u : 0u);
    if (m->fix) {
        put_i24(&f->data[2], sat(m->x_m * 100.0, I24_MIN, I24_MAX));
        put_i24(&f->data[5], sat(m->y_m * 100.0, I24_MIN, I24_MAX));
    }
}
void msg_decode_gnss(const can_frame_t *f, msg_gnss_t *m)
{
    m->fix = (flags_of(f) & 1u) != 0;
    m->x_m = get_i24(&f->data[2]) * 0.01;
    m->y_m = get_i24(&f->data[5]) * 0.01;
}

void msg_encode_engine(can_frame_t *f, const msg_engine_t *m)
{
    begin(f, CAN_ID_ENGINE, (uint8_t)((m->mil ? 1u : 0u) | (m->running ? 2u : 0u)));
    put_u16(&f->data[2], (uint32_t)sat(m->rpm, 0, 65534));
    put_u16(&f->data[4], (uint32_t)sat(m->speed_kph * 100.0, 0, 65534));
}
void msg_decode_engine(const can_frame_t *f, msg_engine_t *m)
{
    m->mil = (flags_of(f) & 1u) != 0;
    m->running = (flags_of(f) & 2u) != 0;
    m->rpm = get_u16(&f->data[2]);
    m->speed_kph = get_u16(&f->data[4]) * 0.01;
}

void msg_encode_inputs(can_frame_t *f, const msg_inputs_t *m)
{
    begin(f, CAN_ID_INPUTS, 0);
    f->data[2] = m->bits & 0x3Fu;
}
void msg_decode_inputs(const can_frame_t *f, msg_inputs_t *m) { m->bits = f->data[2] & 0x3Fu; }

void msg_encode_nav_state(can_frame_t *f, const msg_nav_state_t *m)
{
    begin(f, CAN_ID_NAV_STATE, (uint8_t)(m->status & 3u));
    f->data[2] = m->flags;
    double h = fmod(m->heading_deg, 360.0);
    if (h < 0) h += 360.0;
    uint32_t hc = (uint32_t)sat(h * 100.0, 0, 35999 + 1);
    if (hc >= 36000u) hc = 0;
    put_u16(&f->data[3], hc);
    put_u16(&f->data[5], (uint32_t)sat(m->pos_std_m * 100.0, 0, 65534));
    f->data[7] = (uint8_t)sat(m->gnss_age_s * 10.0, 0, 255);
}
void msg_decode_nav_state(const can_frame_t *f, msg_nav_state_t *m)
{
    m->status = (nav_status_t)(flags_of(f) & 3u);
    m->flags = f->data[2];
    m->heading_deg = get_u16(&f->data[3]) * 0.01;
    m->pos_std_m = get_u16(&f->data[5]) * 0.01;
    m->gnss_age_s = f->data[7] * 0.1;
}

void msg_encode_nav_pose(can_frame_t *f, const msg_nav_pose_t *m)
{
    begin(f, CAN_ID_NAV_POSE, 0);
    put_i24(&f->data[2], sat(m->x_m * 100.0, I24_MIN, I24_MAX));
    put_i24(&f->data[5], sat(m->y_m * 100.0, I24_MIN, I24_MAX));
}
void msg_decode_nav_pose(const can_frame_t *f, msg_nav_pose_t *m)
{
    m->x_m = get_i24(&f->data[2]) * 0.01;
    m->y_m = get_i24(&f->data[5]) * 0.01;
}

void msg_encode_nav_diag(can_frame_t *f, const msg_nav_diag_t *m)
{
    begin(f, CAN_ID_NAV_DIAG, 0);
    put_u16(&f->data[2], (uint32_t)sat(m->speed_mps * 100.0, 0, 65534));
    const int32_t b = sat(m->bias_radps * 1e5, -32768, 32767);
    put_u16(&f->data[4], (uint32_t)(uint16_t)(int16_t)b);
    f->data[6] = (uint8_t)(m->gnss_rejects & 0xFFu);   /* wraps: receiver tracks deltas */
    f->data[7] = m->recoveries;
}
void msg_decode_nav_diag(const can_frame_t *f, msg_nav_diag_t *m)
{
    m->speed_mps = get_u16(&f->data[2]) * 0.01;
    m->bias_radps = (int16_t)get_u16(&f->data[4]) * 1e-5;
    m->gnss_rejects = f->data[6];
    m->recoveries = f->data[7];
}

void msg_encode_nav_cmd(can_frame_t *f, const msg_nav_cmd_t *m)
{
    begin(f, CAN_ID_NAV_CMD, (uint8_t)((m->gating ? 1u : 0u) | (m->realign ? 2u : 0u)));
    put_u16(&f->data[2], m->lockout_rejects);
    put_u16(&f->data[4], (uint32_t)sat((1.0 - m->gate_prob) * 1e6, 1, 65534));
}
void msg_decode_nav_cmd(const can_frame_t *f, msg_nav_cmd_t *m)
{
    m->gating = (flags_of(f) & 1u) != 0;
    m->realign = (flags_of(f) & 2u) != 0;
    m->lockout_rejects = get_u16(&f->data[2]);
    m->gate_prob = 1.0 - get_u16(&f->data[4]) * 1e-6;
}

void msg_encode_cluster_a(can_frame_t *f, const msg_cluster_a_t *m)
{
    begin(f, CAN_ID_CLUSTER_A, 0);
    put_u16(&f->data[2], m->speed_dkph);
    put_u16(&f->data[4], m->rpm);
    f->data[6] = m->lamps;
    f->data[7] = m->lamps2;
}
void msg_decode_cluster_a(const can_frame_t *f, msg_cluster_a_t *m)
{
    m->speed_dkph = get_u16(&f->data[2]);
    m->rpm = get_u16(&f->data[4]);
    m->lamps = f->data[6];
    m->lamps2 = f->data[7];
}

void msg_encode_cluster_b(can_frame_t *f, const msg_cluster_b_t *m)
{
    begin(f, CAN_ID_CLUSTER_B, 0);
    put_u16(&f->data[2], m->heading_cdeg);
    f->data[4] = (uint8_t)m->text;
    put_u16(&f->data[5], m->pos_std_cm);
}
void msg_decode_cluster_b(const can_frame_t *f, msg_cluster_b_t *m)
{
    m->heading_cdeg = get_u16(&f->data[2]);
    m->text = (pos_text_t)f->data[4];
    m->pos_std_cm = get_u16(&f->data[5]);
}
