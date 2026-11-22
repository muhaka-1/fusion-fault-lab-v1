/* M12: the C port of the EKF, the chi-square gate and the fault monitor.
 *      tests/test_parity.py checks it against the Python filter step by step.
 * M13: the nav node (alignment, frames in, NAV_STATE/POSE/DIAG out) on a synthetic drive. */
#include "test.h"
#include "chi2.h"
#include "nav_ekf.h"
#include "nav_monitor.h"
#include "nav_node.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

static void chi2_matches_scipy(void)
{
    /* reference values: scipy.stats.chi2.ppf */
    CHECK_NEAR(chi2_ppf(0.999, 1), 10.827566170662733, 1e-9);
    CHECK_NEAR(chi2_ppf(0.999, 2), 13.815510557964274, 1e-9);
    CHECK_NEAR(chi2_ppf(0.999, 10), 29.588298445074418, 1e-8);
    CHECK_NEAR(chi2_ppf(0.999, 20), 45.314746618125974, 1e-8);
    CHECK_NEAR(chi2_ppf(0.95, 1), 3.841458820694124, 1e-9);
    CHECK_NEAR(chi2_cdf(chi2_ppf(0.99, 7), 7), 0.99, 1e-12);
}

static void wrap_matches_python_modulo(void)
{
    CHECK_NEAR(ekf_wrap(0.0), 0.0, 0);
    CHECK_NEAR(ekf_wrap(PI + 0.1), -PI + 0.1, 1e-12);
    CHECK_NEAR(ekf_wrap(-PI - 0.1), PI - 0.1, 1e-12);
    CHECK_NEAR(ekf_wrap(7 * PI), -PI, 1e-12);       /* Python: ((7pi + pi) % 2pi) - pi = -pi */
}

static void predict_moves_along_heading(void)
{
    ekf_t e;
    const double x0[4] = { 0, 0, PI / 2, 10 };
    ekf_init(&e, x0, NULL, true);
    for (int i = 0; i < 100; ++i) ekf_predict(&e, 0.0, 0.0, 0.01);
    CHECK_NEAR(e.x[0], 0.0, 1e-9);
    CHECK_NEAR(e.x[1], 10.0, 1e-9);
    CHECK(e.P[2][2] > 0.04);                        /* heading uncertainty grows */
    CHECK_NEAR(e.P[0][1], e.P[1][0], 0);            /* stays symmetric */
}

static void gnss_update_shrinks_uncertainty(void)
{
    ekf_t e;
    const double x0[4] = { 0, 0, 0, 10 };
    ekf_init(&e, x0, NULL, true);
    const double before = ekf_pos_std(&e);
    const ekf_update_t u = ekf_update_gnss(&e, 1.0, -1.0);
    CHECK(u.accepted && u.dof == 2);
    CHECK(ekf_pos_std(&e) < 0.5 * before);
    CHECK(e.x[0] > 0.9 && e.x[1] < -0.9);
}

static void outlier_rejected_only_when_gating(void)
{
    ekf_t on, off;
    const double x0[4] = { 0, 0, 0, 10 };
    ekf_init(&on, x0, NULL, true);
    ekf_init(&off, x0, NULL, false);
    for (int i = 0; i < 20; ++i) { ekf_update_gnss(&on, 0, 0); ekf_update_gnss(&off, 0, 0); }
    const ekf_update_t a = ekf_update_gnss(&on, 40, 40), b = ekf_update_gnss(&off, 40, 40);
    CHECK(!a.accepted && a.nis > on.gate_gnss);
    CHECK(b.accepted);
    CHECK(fabs(on.x[0]) < 1e-6 && off.x[0] > 1.0);
}

static void lockout_recovery_after_n_rejections(void)
{
    ekf_config_t c = ekf_default_config();
    c.lockout_rejects = 5;
    ekf_t e;
    const double x0[4] = { 0, 0, 0, 0 };
    ekf_init(&e, x0, &c, true);
    for (int i = 0; i < 20; ++i) ekf_update_gnss(&e, 0, 0);
    const double p_before = e.P[0][0];
    for (int i = 0; i < 4; ++i) ekf_update_gnss(&e, 100, 100);
    CHECK_EQ(e.recoveries, 0);
    ekf_update_gnss(&e, 100, 100);
    CHECK_EQ(e.recoveries, 1);
    CHECK_NEAR(e.P[0][0], p_before * 625.0, p_before * 1e-9);
    CHECK_EQ(e.gnss_rejects, 0);
}

static void lockout_zero_disables_recovery(void)
{
    ekf_config_t c = ekf_default_config();
    c.lockout_rejects = 0;
    ekf_t e;
    const double x0[4] = { 0, 0, 0, 0 };
    ekf_init(&e, x0, &c, true);
    for (int i = 0; i < 20; ++i) ekf_update_gnss(&e, 0, 0);
    for (int i = 0; i < 200; ++i) ekf_update_gnss(&e, 100, 100);
    CHECK_EQ(e.recoveries, 0);
    CHECK_EQ(e.gnss_rejects, 200);
}

static void wheel_update_targets_speed_only(void)
{
    ekf_t e;
    const double x0[4] = { 5, 6, 0.3, 10 };
    ekf_init(&e, x0, NULL, true);
    const ekf_update_t u = ekf_update_wheel(&e, 10.5);
    CHECK(u.accepted && u.dof == 1);
    CHECK(e.x[3] > 10.4);
    CHECK_NEAR(e.x[0], 5.0, 1e-12);                 /* P0 is diagonal: no cross-coupling yet */
}

static void monitor_window_alarm_and_status(void)
{
    monitor_t m;
    monitor_init(&m, NULL);
    for (int i = 0; i < 10; ++i) monitor_on_update(&m, MON_GNSS, i * 0.2, 2.0, true);
    CHECK(!m.alarm[MON_GNSS]);
    CHECK_EQ(monitor_status(&m, 2.0, 1.0), NAV_NOMINAL);
    for (int i = 0; i < 10; ++i) monitor_on_update(&m, MON_GNSS, 2.0 + i * 0.2, 6.0, true);  /* sum 60 > 45.3 */
    CHECK(m.alarm[MON_GNSS]);
    CHECK_EQ(monitor_status(&m, 4.0, 1.0), NAV_DEGRADED);
    CHECK_EQ(monitor_status(&m, 4.0, 6.0), NAV_UNAVAILABLE);
}

static void monitor_gnss_timeout(void)
{
    monitor_t m;
    monitor_init(&m, NULL);
    monitor_on_update(&m, MON_GNSS, 10.0, 1.0, true);
    CHECK_EQ(monitor_status(&m, 10.9, 1.0), NAV_NOMINAL);
    CHECK_EQ(monitor_status(&m, 11.1, 1.0), NAV_DEGRADED);
    monitor_on_update(&m, MON_GNSS, 11.2, 50.0, false);    /* rejected fix does not refresh */
    CHECK_EQ(monitor_status(&m, 11.3, 1.0), NAV_DEGRADED);
}

/* ---- nav node on a synthetic straight drive ---- */
typedef struct {
    nav_node_t n;
    e2e_tx_t ti, tw, tg, tc;
    long k;
    double heading, v, bias;
    double jump;            /* GNSS offset [m] */
    int last_status;
    int publishes;
} drive_t;

static void collect(drive_t *d, int cnt, const can_frame_t *out)
{
    for (int i = 0; i < cnt; ++i) {
        if (out[i].id == CAN_ID_NAV_STATE) {
            msg_nav_state_t s;
            msg_decode_nav_state(&out[i], &s);
            d->last_status = (int)s.status;
            d->publishes++;
        }
    }
}

static void drive_step(drive_t *d, bool send_imu)
{
    can_frame_t f, out[NAV_OUT_MAX];
    const double t = (double)d->k * 0.01;
    const int64_t now = 1000000 + d->k * 10000;
    if (send_imu) {
        msg_imu_t m = { d->bias, 0.0 };
        msg_encode_imu(&f, &m); e2e_protect(&d->ti, (uint16_t)f.id, f.data);
        collect(d, nav_node_on_frame(&d->n, &f, now, out), out);
    }
    if (d->k % 2 == 0) {
        msg_wheel_t w = { d->v };
        msg_encode_wheel(&f, &w); e2e_protect(&d->tw, (uint16_t)f.id, f.data);
        nav_node_on_frame(&d->n, &f, now, out);
    }
    if (d->k % 20 == 0) {
        msg_gnss_t g = { true, d->v * t * cos(d->heading) + d->jump, d->v * t * sin(d->heading) + d->jump };
        msg_encode_gnss(&f, &g); e2e_protect(&d->tg, (uint16_t)f.id, f.data);
        nav_node_on_frame(&d->n, &f, now, out);
    }
    collect(d, nav_node_on_idle(&d->n, now, out), out);
    d->k++;
}

static void drive_init(drive_t *d)
{
    memset(d, 0, sizeof *d);
    nav_node_init(&d->n, NULL);
    d->heading = 30.0 * PI / 180.0;
    d->v = 10.0;
    d->last_status = -1;
}

static void node_aligns_then_tracks(void)
{
    static drive_t d;
    drive_init(&d);
    for (int i = 0; i < 50; ++i) drive_step(&d, true);
    CHECK(!d.n.aligned);
    CHECK_EQ(d.last_status, NAV_INIT);
    for (int i = 0; i < 200; ++i) drive_step(&d, true);
    CHECK(d.n.aligned);
    CHECK_NEAR(d.n.ekf.x[2], d.heading, 0.01);
    CHECK(ekf_pos_std(&d.n.ekf) < d.n.mon.cfg.pos_std_limit_m);   /* no UNAVAILABLE flash after alignment */
    for (int i = 0; i < 1000; ++i) drive_step(&d, true);
    const double t = (double)(d.k - 1) * 0.01;
    CHECK_NEAR(d.n.ekf.x[0], d.v * t * cos(d.heading), 0.05);
    CHECK_NEAR(d.n.ekf.x[1], d.v * t * sin(d.heading), 0.05);
    CHECK_EQ(d.last_status, NAV_NOMINAL);
    CHECK_EQ(d.publishes, 125);                     /* 10 Hz over 12.5 s */
}

static void node_rejects_gnss_jump_and_degrades(void)
{
    static drive_t d;
    drive_init(&d);
    for (int i = 0; i < 1000; ++i) drive_step(&d, true);
    d.jump = 40.0;
    for (int i = 0; i < 300; ++i) drive_step(&d, true);
    CHECK_EQ(d.last_status, NAV_DEGRADED);
    const double t = (double)(d.k - 1) * 0.01;
    CHECK_NEAR(d.n.ekf.x[0], d.v * t * cos(d.heading), 0.2);     /* not pulled by the jump */
    CHECK(d.n.recent_reject_steps > 0);
}

static void node_without_gating_follows_jump(void)
{
    static drive_t d;
    drive_init(&d);
    for (int i = 0; i < 1000; ++i) drive_step(&d, true);
    can_frame_t f, out[NAV_OUT_MAX];
    msg_nav_cmd_t c = { false, false, 50, 0.999 };
    msg_encode_nav_cmd(&f, &c); e2e_protect(&d.tc, (uint16_t)f.id, f.data);
    nav_node_on_frame(&d.n, &f, 0, out);
    CHECK(!d.n.ekf.gating);
    d.jump = 40.0;
    for (int i = 0; i < 300; ++i) drive_step(&d, true);
    const double t = (double)(d.k - 1) * 0.01;
    CHECK(hypot(d.n.ekf.x[0] - d.v * t * cos(d.heading), d.n.ekf.x[1] - d.v * t * sin(d.heading)) > 10.0);
}

static void node_reports_imu_loss(void)
{
    static drive_t d;
    drive_init(&d);
    for (int i = 0; i < 400; ++i) drive_step(&d, true);
    for (int i = 0; i < 30; ++i) drive_step(&d, false);       /* 300 ms without IMU */
    CHECK(d.n.imu_lost);
    CHECK_EQ(d.last_status, NAV_UNAVAILABLE);
    for (int i = 0; i < 20; ++i) drive_step(&d, true);
    CHECK(!d.n.imu_lost);
}

static void node_realigns_after_long_imu_loss(void)
{
    /* 1.2 s without IMU = 120 lost frames, but the counter only counts to 14: the node can't know
     * how far the car moved, so it must not just carry on predicting. It aligns again. */
    static drive_t d;
    drive_init(&d);
    for (int i = 0; i < 400; ++i) drive_step(&d, true);
    CHECK_EQ(d.last_status, NAV_NOMINAL);
    for (int i = 0; i < 120; ++i) drive_step(&d, false);
    CHECK_EQ(d.last_status, NAV_UNAVAILABLE);
    for (int i = 0; i < 10; ++i) drive_step(&d, true);
    CHECK(!d.n.aligned);
    CHECK_EQ(d.last_status, NAV_INIT);                       /* ALIGNING, not a wrong position */
    for (int i = 0; i < 300; ++i) drive_step(&d, true);
    const double t = (double)(d.k - 1) * 0.01;
    CHECK(d.n.aligned);
    CHECK_EQ(d.last_status, NAV_NOMINAL);
    CHECK_NEAR(d.n.ekf.x[0], d.v * t * cos(d.heading), 0.5);
    CHECK_NEAR(d.n.ekf.x[1], d.v * t * sin(d.heading), 0.5);
}

static void node_drops_corrupted_frames(void)
{
    static drive_t d;
    drive_init(&d);
    for (int i = 0; i < 400; ++i) drive_step(&d, true);
    const long steps = d.n.steps;
    can_frame_t f, out[NAV_OUT_MAX];
    msg_imu_t m = { 5.0, 0 };                      /* would spin the heading if accepted */
    msg_encode_imu(&f, &m); e2e_protect(&d.ti, (uint16_t)f.id, f.data);
    f.data[3] ^= 0x40;
    const double psi = d.n.ekf.x[2];
    nav_node_on_frame(&d.n, &f, 0, out);
    CHECK_EQ(d.n.steps, steps);
    CHECK_NEAR(d.n.ekf.x[2], psi, 0);
    CHECK_EQ(d.n.e2e_errors, 1);
}

static void node_counts_lost_imu_frames_into_dt(void)
{
    static drive_t d;
    drive_init(&d);
    for (int i = 0; i < 400; ++i) drive_step(&d, true);
    const long steps = d.n.steps;
    drive_step(&d, false);
    drive_step(&d, false);
    drive_step(&d, true);
    CHECK_EQ(d.n.steps, steps + 1);                /* sender counter did not advance while silent */
    can_frame_t f, out[NAV_OUT_MAX];
    msg_imu_t m = { 0, 0 };
    msg_encode_imu(&f, &m); e2e_protect(&d.ti, (uint16_t)f.id, f.data);   /* this one is lost */
    msg_encode_imu(&f, &m); e2e_protect(&d.ti, (uint16_t)f.id, f.data);
    nav_node_on_frame(&d.n, &f, 0, out);
    CHECK_EQ(d.n.steps, steps + 3);
}

void suite_nav(void)
{
    RUN(chi2_matches_scipy);
    RUN(wrap_matches_python_modulo);
    RUN(predict_moves_along_heading);
    RUN(gnss_update_shrinks_uncertainty);
    RUN(outlier_rejected_only_when_gating);
    RUN(lockout_recovery_after_n_rejections);
    RUN(lockout_zero_disables_recovery);
    RUN(wheel_update_targets_speed_only);
    RUN(monitor_window_alarm_and_status);
    RUN(monitor_gnss_timeout);
    RUN(node_aligns_then_tracks);
    RUN(node_rejects_gnss_jump_and_degrades);
    RUN(node_without_gating_follows_jump);
    RUN(node_reports_imu_loss);
    RUN(node_realigns_after_long_imu_loss);
    RUN(node_drops_corrupted_frames);
    RUN(node_counts_lost_imu_frames_into_dt);
}
