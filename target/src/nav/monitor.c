#include "nav_monitor.h"
#include "chi2.h"
#include <string.h>

monitor_config_t monitor_default_config(void)
{
    monitor_config_t c = { .window = 10, .alarm_prob = 0.999, .gnss_timeout_s = 1.0, .pos_std_limit_m = 5.0 };
    return c;
}

void monitor_init(monitor_t *m, const monitor_config_t *cfg)
{
    memset(m, 0, sizeof *m);
    m->cfg = cfg ? *cfg : monitor_default_config();
    if (m->cfg.window < 1) m->cfg.window = 1;
    if (m->cfg.window > MON_MAX_WINDOW) m->cfg.window = MON_MAX_WINDOW;
    const int dof[MON_SENSORS] = { 2, 1 };
    for (int s = 0; s < MON_SENSORS; ++s) m->thr[s] = chi2_ppf(m->cfg.alarm_prob, dof[s] * m->cfg.window);
    m->last_gnss_ok = 0.0;
}

void monitor_on_update(monitor_t *m, int sensor, double t, double nis, bool accepted)
{
    const int w = m->cfg.window;
    m->buf[sensor][m->head[sensor]] = nis;
    m->head[sensor] = (m->head[sensor] + 1) % w;
    if (m->len[sensor] < w) m->len[sensor]++;
    if (sensor == MON_GNSS && accepted) m->last_gnss_ok = t;
    if (m->len[sensor] == w) {
        /* sum oldest-first, like Python's sum() over the deque */
        double sum = 0.0;
        for (int k = 0; k < w; ++k) sum += m->buf[sensor][(m->head[sensor] + k) % w];
        m->alarm[sensor] = sum > m->thr[sensor];
    }
}

nav_status_t monitor_status(const monitor_t *m, double t, double pos_std)
{
    if (pos_std > m->cfg.pos_std_limit_m) return NAV_UNAVAILABLE;
    if ((t - m->last_gnss_ok) > m->cfg.gnss_timeout_s || m->alarm[MON_GNSS] || m->alarm[MON_WHEEL])
        return NAV_DEGRADED;
    return NAV_NOMINAL;
}
