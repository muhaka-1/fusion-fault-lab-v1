/* EKF, state x = [x, y, psi, v, b_g] (C port of fusionlab/ekf.py; keep the two in step).
 * Prediction: IMU yaw rate + longitudinal acceleration as control input.
 * Updates: GNSS position (2 dof), wheel speed (1 dof), each gated on the NIS.
 */
#ifndef NAV_EKF_H
#define NAV_EKF_H
#include <stdbool.h>

#define EKF_N 5

typedef struct {
    double sigma_gyro, sigma_acc, q_bias, sigma_gnss, sigma_wheel, gate_prob;
    double p0_pos, p0_psi, p0_v, p0_bias;
    int lockout_rejects;       /* 0 disables recovery */
    double lockout_inflate;
} ekf_config_t;

typedef struct {
    ekf_config_t cfg;
    bool gating;
    double x[EKF_N];
    double P[EKF_N][EKF_N];
    int gnss_rejects;
    int recoveries;
    double gate_gnss, gate_wheel;   /* cached chi-square thresholds */
} ekf_t;

typedef struct { bool accepted; double nis; int dof; } ekf_update_t;

ekf_config_t ekf_default_config(void);
void ekf_init(ekf_t *e, const double x0[4], const ekf_config_t *cfg, bool gating);
void ekf_set_gate(ekf_t *e, double gate_prob);
void ekf_predict(ekf_t *e, double gyro, double acc, double dt);
ekf_update_t ekf_update_gnss(ekf_t *e, double zx, double zy);
ekf_update_t ekf_update_wheel(ekf_t *e, double z);
double ekf_pos_std(const ekf_t *e);
double ekf_wrap(double a);

#endif
