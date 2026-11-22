/* Navigation node: everything the nav process does, without I/O, so it can be unit tested.
 * Time base is the IMU sample count (100 Hz), not the wall clock, so a run is deterministic.
 *
 * Start-up: the EKF needs a heading. While ALIGNING (status NAV_INIT) the node integrates
 * the gyro and collects GNSS fixes; once two fixes are >= 1 s and >= 5 m apart it takes the
 * chord direction, corrects it by the gyro-integrated turn since the chord's midpoint, and
 * starts the filter at the latest fix and wheel speed.
 *
 * IMU loss: silent for imu_timeout_us -> UNAVAILABLE + IMU_LOST. When the IMU comes back the node
 * aligns again: after such a gap the alive counter can't tell how much time has passed (M13).
 */
#ifndef NAV_NODE_H
#define NAV_NODE_H
#include <stdbool.h>
#include <stdint.h>
#include "can_msg.h"
#include "e2e.h"
#include "nav_ekf.h"
#include "nav_monitor.h"

#define NAV_OUT_MAX 4
#define NAV_ALIGN_FIXES 16

typedef struct {
    double dt;                 /* IMU period [s] */
    int publish_every;         /* IMU steps between outputs (10 -> 10 Hz) */
    int64_t imu_timeout_us;    /* wall-clock IMU silence -> UNAVAILABLE */
    ekf_config_t ekf;
    monitor_config_t mon;
    bool gating;
} nav_node_config_t;

typedef struct {
    nav_node_config_t cfg;
    ekf_t ekf;
    monitor_t mon;
    bool aligned;
    long steps;                /* IMU steps processed (incl. lost ones) */
    double t;                  /* sim time = steps * dt */
    double psi_int;            /* integrated gyro, for alignment */
    double last_wheel;
    bool have_wheel;
    struct { double t, x, y, psi_int; } fix[NAV_ALIGN_FIXES];
    int nfix;
    e2e_rx_t rx_imu, rx_wheel, rx_gnss, rx_cmd;
    e2e_tx_t tx_state, tx_pose, tx_diag;
    int64_t last_imu_us, last_idle_pub_us;
    bool imu_lost;
    uint32_t e2e_errors;       /* discarded sensor frames */
    int recent_reject_steps;   /* >0: a GNSS fix was rejected in the last second */
    nav_status_t status;
} nav_node_t;

nav_node_config_t nav_node_default_config(void);
void nav_node_init(nav_node_t *n, const nav_node_config_t *cfg);
/* feed one bus frame; returns number of frames written to out[] (to be sent) */
int nav_node_on_frame(nav_node_t *n, const can_frame_t *f, int64_t now_us, can_frame_t out[NAV_OUT_MAX]);
/* call periodically; publishes UNAVAILABLE + IMU_LOST when the IMU has gone silent */
int nav_node_on_idle(nav_node_t *n, int64_t now_us, can_frame_t out[NAV_OUT_MAX]);
void nav_node_realign(nav_node_t *n);

#endif
