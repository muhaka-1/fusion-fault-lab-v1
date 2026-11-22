/* CAN message set of the bench. Single source of truth: docs/can_messages.md. Python mirror: src/fusionlab/canbus.py.
 * All frames are 8 bytes, little endian, E2E-protected (see e2e.h):
 * byte 0 = CRC-8, byte 1 low nibble = alive counter, high nibble = flags.
 */
#ifndef CAN_MSG_H
#define CAN_MSG_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

enum {
    CAN_ID_IMU = 0x100,        /* sensors -> nav, 100 Hz */
    CAN_ID_WHEEL = 0x110,      /* sensors -> nav, 50 Hz */
    CAN_ID_GNSS = 0x120,       /* sensors -> nav, 5 Hz */
    CAN_ID_ENGINE = 0x200,     /* ECU -> core, 50 Hz */
    CAN_ID_INPUTS = 0x210,     /* input board (stalks, switches) -> core, 100 Hz */
    CAN_ID_NAV_STATE = 0x300,  /* nav -> core, 10 Hz */
    CAN_ID_NAV_POSE = 0x301,   /* nav -> anyone, 10 Hz */
    CAN_ID_NAV_DIAG = 0x302,   /* nav -> anyone, 10 Hz */
    CAN_ID_NAV_CMD = 0x310,    /* bench -> nav, on change: filter settings, re-align */
    CAN_ID_CLUSTER_A = 0x400,  /* core -> display, 20 Hz: gauges + lamps */
    CAN_ID_CLUSTER_B = 0x401   /* core -> display, 20 Hz: positioning */
};

#define CAN_U16_INVALID 0xFFFFu

typedef struct { double gyro_radps, acc_mps2; } msg_imu_t;
typedef struct { double speed_mps; } msg_wheel_t;
typedef struct { bool fix; double x_m, y_m; } msg_gnss_t;
typedef struct { bool mil, running; double rpm, speed_kph; } msg_engine_t;

enum { IN_STALK_LEFT = 1u << 0, IN_STALK_RIGHT = 1u << 1, IN_HAZARD_BTN = 1u << 2,
       IN_DIP_SWITCH = 1u << 3, IN_MAIN_BEAM = 1u << 4, IN_FLASH = 1u << 5 };
typedef struct { uint8_t bits; } msg_inputs_t;

typedef enum { NAV_NOMINAL = 0, NAV_DEGRADED = 1, NAV_UNAVAILABLE = 2, NAV_INIT = 3 } nav_status_t;
enum { NAVF_GNSS_ALARM = 1u << 0, NAVF_WHEEL_ALARM = 1u << 1, NAVF_GNSS_REJECTING = 1u << 2,
       NAVF_IMU_LOST = 1u << 3, NAVF_RECOVERY = 1u << 4, NAVF_INPUT_E2E_ERR = 1u << 5 };
typedef struct {
    nav_status_t status;
    uint8_t flags;
    double heading_deg;   /* 0..360 */
    double pos_std_m;
    double gnss_age_s;
} msg_nav_state_t;
typedef struct { double x_m, y_m; } msg_nav_pose_t;
typedef struct { double speed_mps, bias_radps; uint16_t gnss_rejects; uint8_t recoveries; } msg_nav_diag_t;
typedef struct {
    bool gating, realign;
    uint16_t lockout_rejects;   /* 0 = recovery off */
    double gate_prob;           /* sent as (1 - p) in ppm */
} msg_nav_cmd_t;

enum { LAMP_LEFT = 1u << 0, LAMP_RIGHT = 1u << 1, LAMP_DIP = 1u << 2, LAMP_MAIN = 1u << 3,
       LAMP_CHECK_ENGINE = 1u << 4, LAMP_ECU_FAULT = 1u << 5, LAMP_POS_AMBER = 1u << 6, LAMP_POS_RED = 1u << 7 };
enum { LAMP2_INPUT_FAULT = 1u << 0, LAMP2_HAZARD = 1u << 1 };
typedef struct {
    uint16_t speed_dkph;  /* 0.1 km/h, CAN_U16_INVALID = show "--" */
    uint16_t rpm;         /* CAN_U16_INVALID = show "--" */
    uint8_t lamps, lamps2;
} msg_cluster_a_t;
typedef enum { POS_TXT_OK = 0, POS_TXT_DEGRADED, POS_TXT_UNAVAILABLE, POS_TXT_ALIGNING, POS_TXT_NAV_LOST } pos_text_t;
typedef struct {
    uint16_t heading_cdeg; /* 0.01 deg, CAN_U16_INVALID = "--" */
    pos_text_t text;
    uint16_t pos_std_cm;   /* CAN_U16_INVALID = unknown */
} msg_cluster_b_t;

/* encode: fills id/dlc/data[2..7] and flag nibble; caller applies e2e_protect() */
void msg_encode_imu(can_frame_t *f, const msg_imu_t *m);
void msg_encode_wheel(can_frame_t *f, const msg_wheel_t *m);
void msg_encode_gnss(can_frame_t *f, const msg_gnss_t *m);
void msg_encode_engine(can_frame_t *f, const msg_engine_t *m);
void msg_encode_inputs(can_frame_t *f, const msg_inputs_t *m);
void msg_encode_nav_state(can_frame_t *f, const msg_nav_state_t *m);
void msg_encode_nav_pose(can_frame_t *f, const msg_nav_pose_t *m);
void msg_encode_nav_diag(can_frame_t *f, const msg_nav_diag_t *m);
void msg_encode_nav_cmd(can_frame_t *f, const msg_nav_cmd_t *m);
void msg_encode_cluster_a(can_frame_t *f, const msg_cluster_a_t *m);
void msg_encode_cluster_b(can_frame_t *f, const msg_cluster_b_t *m);

void msg_decode_imu(const can_frame_t *f, msg_imu_t *m);
void msg_decode_wheel(const can_frame_t *f, msg_wheel_t *m);
void msg_decode_gnss(const can_frame_t *f, msg_gnss_t *m);
void msg_decode_engine(const can_frame_t *f, msg_engine_t *m);
void msg_decode_inputs(const can_frame_t *f, msg_inputs_t *m);
void msg_decode_nav_state(const can_frame_t *f, msg_nav_state_t *m);
void msg_decode_nav_pose(const can_frame_t *f, msg_nav_pose_t *m);
void msg_decode_nav_diag(const can_frame_t *f, msg_nav_diag_t *m);
void msg_decode_nav_cmd(const can_frame_t *f, msg_nav_cmd_t *m);
void msg_decode_cluster_a(const can_frame_t *f, msg_cluster_a_t *m);
void msg_decode_cluster_b(const can_frame_t *f, msg_cluster_b_t *m);

#endif
