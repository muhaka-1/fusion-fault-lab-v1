/* Fault monitor (C port of fusionlab/monitor.py): windowed NIS alarm per sensor and
 * NOMINAL / DEGRADED / UNAVAILABLE status. */
#ifndef NAV_MONITOR_H
#define NAV_MONITOR_H
#include <stdbool.h>
#include "can_msg.h"

#define MON_MAX_WINDOW 200
enum { MON_GNSS = 0, MON_WHEEL = 1, MON_SENSORS = 2 };

typedef struct {
    int window;
    double alarm_prob, gnss_timeout_s, pos_std_limit_m;
} monitor_config_t;

typedef struct {
    monitor_config_t cfg;
    double buf[MON_SENSORS][MON_MAX_WINDOW];
    int len[MON_SENSORS], head[MON_SENSORS];
    double thr[MON_SENSORS];
    bool alarm[MON_SENSORS];
    double last_gnss_ok;
} monitor_t;

monitor_config_t monitor_default_config(void);
void monitor_init(monitor_t *m, const monitor_config_t *cfg);
void monitor_on_update(monitor_t *m, int sensor, double t, double nis, bool accepted);
nav_status_t monitor_status(const monitor_t *m, double t, double pos_std);

#endif
