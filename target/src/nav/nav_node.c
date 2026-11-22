#include "nav_node.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846
#define ALIGN_MIN_DT 1.0
#define ALIGN_MIN_DIST 5.0

nav_node_config_t nav_node_default_config(void)
{
    nav_node_config_t c = {
        .dt = 0.01, .publish_every = 10, .imu_timeout_us = 100000,
        .ekf = ekf_default_config(), .mon = monitor_default_config(), .gating = true,
    };
    return c;
}

void nav_node_realign(nav_node_t *n)
{
    n->aligned = false;
    n->nfix = 0;
    n->psi_int = 0.0;
    n->status = NAV_INIT;
}

void nav_node_init(nav_node_t *n, const nav_node_config_t *cfg)
{
    memset(n, 0, sizeof *n);
    n->cfg = cfg ? *cfg : nav_node_default_config();
    e2e_rx_init(&n->rx_imu, 14, 0);
    e2e_rx_init(&n->rx_wheel, 14, 0);
    e2e_rx_init(&n->rx_gnss, 14, 0);
    e2e_rx_init(&n->rx_cmd, 14, 0);
    monitor_init(&n->mon, &n->cfg.mon);
    nav_node_realign(n);
}

static bool sensor_ok(nav_node_t *n, e2e_rx_t *rx, const can_frame_t *f)
{
    const e2e_status_t s = e2e_check(rx, (uint16_t)f->id, f->data);
    /* sensor data: the first frame after start or resync is used too */
    if (e2e_usable(s) || s == E2E_INITIAL) return true;
    n->e2e_errors++;
    return false;
}

static void try_align(nav_node_t *n)
{
    if (n->nfix < 2 || !n->have_wheel) return;
    const int last = n->nfix - 1;
    for (int i = 0; i < last; ++i) {
        const double dt = n->fix[last].t - n->fix[i].t;
        const double dx = n->fix[last].x - n->fix[i].x, dy = n->fix[last].y - n->fix[i].y;
        if (dt >= ALIGN_MIN_DT && hypot(dx, dy) >= ALIGN_MIN_DIST) {
            const double mid = 0.5 * (n->fix[i].psi_int + n->fix[last].psi_int);
            const double heading = ekf_wrap(atan2(dy, dx) + (n->fix[last].psi_int - mid));
            const double x0[4] = { n->fix[last].x, n->fix[last].y, heading, n->last_wheel };
            ekf_init(&n->ekf, x0, &n->cfg.ekf, n->cfg.gating);
            /* started from a GNSS fix: position is as good as one fix, not the generic p0_pos */
            n->ekf.P[0][0] = n->ekf.P[1][1] = n->cfg.ekf.sigma_gnss * n->cfg.ekf.sigma_gnss;
            monitor_init(&n->mon, &n->cfg.mon);
            n->mon.last_gnss_ok = n->t;
            n->aligned = true;
            n->status = monitor_status(&n->mon, n->t, ekf_pos_std(&n->ekf));
            return;
        }
    }
}

static void push_fix(nav_node_t *n, double x, double y)
{
    if (n->nfix == NAV_ALIGN_FIXES) {
        memmove(&n->fix[0], &n->fix[1], sizeof n->fix[0] * (NAV_ALIGN_FIXES - 1));
        n->nfix--;
    }
    n->fix[n->nfix].t = n->t;
    n->fix[n->nfix].x = x;
    n->fix[n->nfix].y = y;
    n->fix[n->nfix].psi_int = n->psi_int;
    n->nfix++;
}

static int publish(nav_node_t *n, can_frame_t out[NAV_OUT_MAX])
{
    msg_nav_state_t st = { .status = n->status, .flags = 0, .heading_deg = 0, .pos_std_m = 655.34, .gnss_age_s = 25.5 };
    msg_nav_pose_t pose = { 0, 0 };
    msg_nav_diag_t diag = { 0, 0, 0, 0 };
    if (n->aligned) {
        st.heading_deg = n->ekf.x[2] * 180.0 / PI;
        st.pos_std_m = ekf_pos_std(&n->ekf);
        st.gnss_age_s = n->t - n->mon.last_gnss_ok;
        if (n->mon.alarm[MON_GNSS]) st.flags |= NAVF_GNSS_ALARM;
        if (n->mon.alarm[MON_WHEEL]) st.flags |= NAVF_WHEEL_ALARM;
        if (n->recent_reject_steps > 0) st.flags |= NAVF_GNSS_REJECTING;
        if (n->ekf.recoveries > 0) st.flags |= NAVF_RECOVERY;
        pose.x_m = n->ekf.x[0];
        pose.y_m = n->ekf.x[1];
        diag.speed_mps = n->ekf.x[3];
        diag.bias_radps = n->ekf.x[4];
        diag.gnss_rejects = (uint16_t)n->ekf.gnss_rejects;
        diag.recoveries = (uint8_t)(n->ekf.recoveries > 255 ? 255 : n->ekf.recoveries);
    }
    if (n->imu_lost) { st.status = NAV_UNAVAILABLE; st.flags |= NAVF_IMU_LOST; }
    if (n->e2e_errors) st.flags |= NAVF_INPUT_E2E_ERR;
    msg_encode_nav_state(&out[0], &st); e2e_protect(&n->tx_state, (uint16_t)out[0].id, out[0].data);
    msg_encode_nav_pose(&out[1], &pose); e2e_protect(&n->tx_pose, (uint16_t)out[1].id, out[1].data);
    msg_encode_nav_diag(&out[2], &diag); e2e_protect(&n->tx_diag, (uint16_t)out[2].id, out[2].data);
    n->e2e_errors = 0;
    return 3;
}

int nav_node_on_frame(nav_node_t *n, const can_frame_t *f, int64_t now_us, can_frame_t out[NAV_OUT_MAX])
{
    switch (f->id) {
    case CAN_ID_IMU: {
        if (!sensor_ok(n, &n->rx_imu, f)) return 0;
        msg_imu_t m;
        msg_decode_imu(f, &m);
        const bool first = n->steps == 0 && !n->last_imu_us;
        int k = first ? 0 : n->rx_imu.last_delta;           /* >1: IMU frames were lost */
        if (n->imu_lost) {
            /* The IMU was silent longer than imu_timeout_us (100 ms = 10 frames). The alive counter
             * wraps every 15 frames, so after such a gap it can't say how many frames were lost:
             * the elapsed time is unknown, and predicting across it would put the filter in the
             * wrong place. Estimate the steps from the wall clock and start again from alignment. */
            const double gap = (double)(now_us - n->last_imu_us) / (n->cfg.dt * 1e6);
            k = gap < 1.0 ? 1 : (int)(gap + 0.5);
            nav_node_realign(n);
        }
        n->last_imu_us = now_us;
        n->imu_lost = false;
        n->steps += k;
        n->t = (double)n->steps * n->cfg.dt;
        const double dt = k * n->cfg.dt;
        if (!n->aligned) {
            n->psi_int += m.gyro_radps * dt;
        } else if (k > 0) {
            ekf_predict(&n->ekf, m.gyro_radps, m.acc_mps2, dt);
            n->status = monitor_status(&n->mon, n->t, ekf_pos_std(&n->ekf));
        }
        if (n->recent_reject_steps > 0) n->recent_reject_steps -= k;
        if (n->steps / n->cfg.publish_every != (n->steps - k) / n->cfg.publish_every || first)
            return publish(n, out);
        return 0;
    }
    case CAN_ID_WHEEL: {
        if (!sensor_ok(n, &n->rx_wheel, f)) return 0;
        msg_wheel_t m;
        msg_decode_wheel(f, &m);
        n->last_wheel = m.speed_mps;
        n->have_wheel = true;
        if (n->aligned) {
            const ekf_update_t u = ekf_update_wheel(&n->ekf, m.speed_mps);
            monitor_on_update(&n->mon, MON_WHEEL, n->t, u.nis, u.accepted);
            n->status = monitor_status(&n->mon, n->t, ekf_pos_std(&n->ekf));
        }
        return 0;
    }
    case CAN_ID_GNSS: {
        if (!sensor_ok(n, &n->rx_gnss, f)) return 0;
        msg_gnss_t m;
        msg_decode_gnss(f, &m);
        if (!m.fix) return 0;                        /* receiver reports no fix */
        if (!n->aligned) {
            push_fix(n, m.x_m, m.y_m);
            try_align(n);
        } else {
            const ekf_update_t u = ekf_update_gnss(&n->ekf, m.x_m, m.y_m);
            monitor_on_update(&n->mon, MON_GNSS, n->t, u.nis, u.accepted);
            if (!u.accepted) n->recent_reject_steps = (int)(1.0 / n->cfg.dt);
            n->status = monitor_status(&n->mon, n->t, ekf_pos_std(&n->ekf));
        }
        return 0;
    }
    case CAN_ID_NAV_CMD: {
        const e2e_status_t s = e2e_check(&n->rx_cmd, (uint16_t)f->id, f->data);
        if (!(e2e_usable(s) || s == E2E_INITIAL)) return 0;
        msg_nav_cmd_t c;
        msg_decode_nav_cmd(f, &c);
        n->cfg.gating = c.gating;
        n->cfg.ekf.lockout_rejects = c.lockout_rejects;
        if (c.gate_prob > 0.5 && c.gate_prob < 1.0) n->cfg.ekf.gate_prob = c.gate_prob;
        n->ekf.gating = c.gating;
        n->ekf.cfg.lockout_rejects = c.lockout_rejects;
        ekf_set_gate(&n->ekf, n->cfg.ekf.gate_prob);
        if (c.realign) nav_node_realign(n);
        return 0;
    }
    default:
        return 0;
    }
}

int nav_node_on_idle(nav_node_t *n, int64_t now_us, can_frame_t out[NAV_OUT_MAX])
{
    if (!n->last_imu_us || now_us - n->last_imu_us < n->cfg.imu_timeout_us) return 0;
    n->imu_lost = true;
    if (now_us - n->last_idle_pub_us < 100000) return 0;
    n->last_idle_pub_us = now_us;
    return publish(n, out);
}
