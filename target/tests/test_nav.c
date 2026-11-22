/* M12: the C port of the EKF, the chi-square gate and the fault monitor.
 * tests/test_parity.py checks it against the Python filter step by step. */
#include "test.h"
#include "chi2.h"
#include "nav_ekf.h"
#include "nav_monitor.h"
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
}
