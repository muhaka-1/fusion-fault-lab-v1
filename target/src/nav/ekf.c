#include "nav_ekf.h"
#include "chi2.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

double ekf_wrap(double a)
{
    /* same as Python ((a + pi) % (2 pi)) - pi, including the sign rule of % */
    const double w = 2.0 * PI;
    double r = fmod(a + PI, w);
    if (r != 0.0 && r < 0.0) r += w;
    return r - PI;
}

ekf_config_t ekf_default_config(void)
{
    ekf_config_t c = {
        .sigma_gyro = 0.003, .sigma_acc = 0.05, .q_bias = 1e-5, .sigma_gnss = 1.5, .sigma_wheel = 0.10,
        .gate_prob = 0.999, .p0_pos = 5.0, .p0_psi = 0.2, .p0_v = 1.0, .p0_bias = 0.02,
        .lockout_rejects = 50, .lockout_inflate = 25.0,
    };
    return c;
}

void ekf_set_gate(ekf_t *e, double gate_prob)
{
    e->cfg.gate_prob = gate_prob;
    e->gate_gnss = chi2_ppf(gate_prob, 2);
    e->gate_wheel = chi2_ppf(gate_prob, 1);
}

void ekf_init(ekf_t *e, const double x0[4], const ekf_config_t *cfg, bool gating)
{
    memset(e, 0, sizeof *e);
    e->cfg = cfg ? *cfg : ekf_default_config();
    e->gating = gating;
    for (int i = 0; i < 4; ++i) e->x[i] = x0[i];
    e->x[4] = 0.0;
    const ekf_config_t *c = &e->cfg;
    e->P[0][0] = c->p0_pos * c->p0_pos;
    e->P[1][1] = c->p0_pos * c->p0_pos;
    e->P[2][2] = c->p0_psi * c->p0_psi;
    e->P[3][3] = c->p0_v * c->p0_v;
    e->P[4][4] = c->p0_bias * c->p0_bias;
    ekf_set_gate(e, c->gate_prob);
}

static void copy_into(double dst[EKF_N][EKF_N], const double src[EKF_N][EKF_N])
{
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) dst[i][j] = src[i][j];
}

static void symmetrize(double P[EKF_N][EKF_N])
{
    for (int i = 0; i < EKF_N; ++i)
        for (int j = i + 1; j < EKF_N; ++j) {
            const double m = 0.5 * (P[i][j] + P[j][i]);
            P[i][j] = m; P[j][i] = m;
        }
}

void ekf_predict(ekf_t *e, double gyro, double acc, double dt)
{
    const ekf_config_t *c = &e->cfg;
    const double psi = e->x[2], v = e->x[3], b = e->x[4];
    const double cs = cos(psi), sn = sin(psi), w = gyro - b;
    e->x[0] += v * cs * dt;
    e->x[1] += v * sn * dt;
    e->x[2] = ekf_wrap(psi + w * dt);
    e->x[3] = v + acc * dt;

    double F[EKF_N][EKF_N] = {{0}};
    for (int i = 0; i < EKF_N; ++i) F[i][i] = 1.0;
    F[0][2] = -v * sn * dt; F[0][3] = cs * dt;
    F[1][2] = v * cs * dt;  F[1][3] = sn * dt;
    F[2][4] = -dt;

    double FP[EKF_N][EKF_N], Pn[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += F[i][k] * e->P[k][j];
            FP[i][j] = s;
        }
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += FP[i][k] * F[j][k];
            Pn[i][j] = s;
        }
    Pn[2][2] += c->sigma_gyro * c->sigma_gyro * dt * dt;
    Pn[3][3] += c->sigma_acc * c->sigma_acc * dt * dt;
    Pn[4][4] += c->q_bias * c->q_bias * dt;
    copy_into(e->P, (const double (*)[EKF_N])Pn);
    symmetrize(e->P);
}

/* Gated update for a measurement of m <= 2 states selected by idx (H rows are unit vectors). */
static ekf_update_t update(ekf_t *e, int m, const int idx[2], const double z[2], double r, double gate)
{
    double y[2], S[2][2];
    for (int i = 0; i < m; ++i) {
        y[i] = z[i] - e->x[idx[i]];
        for (int j = 0; j < m; ++j) S[i][j] = e->P[idx[i]][idx[j]] + (i == j ? r : 0.0);
    }
    double Si[2][2];
    if (m == 1) {
        Si[0][0] = 1.0 / S[0][0];
    } else {
        const double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        Si[0][0] = S[1][1] / det; Si[1][1] = S[0][0] / det;
        Si[0][1] = -S[0][1] / det; Si[1][0] = -S[1][0] / det;
    }
    double nis = 0.0;
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < m; ++j) nis += y[i] * Si[i][j] * y[j];

    ekf_update_t out = { .accepted = !e->gating || nis <= gate, .nis = nis, .dof = m };
    if (!out.accepted) return out;

    double K[EKF_N][2];
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < m; ++j) {
            double s = 0.0;
            for (int k = 0; k < m; ++k) s += e->P[i][idx[k]] * Si[k][j];
            K[i][j] = s;
        }
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < m; ++j) e->x[i] += K[i][j] * y[j];
    e->x[2] = ekf_wrap(e->x[2]);

    /* Joseph form: P = (I - KH) P (I - KH)^T + K R K^T */
    double A[EKF_N][EKF_N], AP[EKF_N][EKF_N], Pn[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double kh = 0.0;
            for (int k = 0; k < m; ++k) if (idx[k] == j) kh += K[i][k];
            A[i][j] = (i == j ? 1.0 : 0.0) - kh;
        }
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += A[i][k] * e->P[k][j];
            AP[i][j] = s;
        }
    for (int i = 0; i < EKF_N; ++i)
        for (int j = 0; j < EKF_N; ++j) {
            double s = 0.0;
            for (int k = 0; k < EKF_N; ++k) s += AP[i][k] * A[j][k];
            for (int k = 0; k < m; ++k) s += K[i][k] * r * K[j][k];
            Pn[i][j] = s;
        }
    copy_into(e->P, (const double (*)[EKF_N])Pn);
    symmetrize(e->P);
    return out;
}

ekf_update_t ekf_update_gnss(ekf_t *e, double zx, double zy)
{
    static const int idx[2] = { 0, 1 };
    const double z[2] = { zx, zy };
    const ekf_update_t out = update(e, 2, idx, z, e->cfg.sigma_gnss * e->cfg.sigma_gnss, e->gate_gnss);
    e->gnss_rejects = out.accepted ? 0 : e->gnss_rejects + 1;
    if (e->gating && e->cfg.lockout_rejects > 0 && e->gnss_rejects >= e->cfg.lockout_rejects) {
        /* Lock-out recovery: rejected GNSS for so long that the filter's own estimate is
           more likely wrong than the sensor; inflate pos/heading/bias so the gate re-opens. */
        const double f = e->cfg.lockout_inflate;
        const double d[EKF_N] = { f, f, f, 1.0, f };
        for (int i = 0; i < EKF_N; ++i)
            for (int j = 0; j < EKF_N; ++j) e->P[i][j] *= d[i] * d[j];
        e->gnss_rejects = 0;
        e->recoveries++;
    }
    return out;
}

ekf_update_t ekf_update_wheel(ekf_t *e, double z)
{
    static const int idx[2] = { 3, 0 };
    const double zz[2] = { z, 0.0 };
    return update(e, 1, idx, zz, e->cfg.sigma_wheel * e->cfg.sigma_wheel, e->gate_wheel);
}

double ekf_pos_std(const ekf_t *e) { return sqrt(e->P[0][0] + e->P[1][1]); }
