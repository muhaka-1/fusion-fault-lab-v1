/* nav: sensor-fusion process. Reads IMU / wheel / GNSS frames, runs the EKF with fault
 * detection, publishes NAV_STATE / NAV_POSE / NAV_DIAG at 10 Hz. */
#include "canbus.h"
#include "clock.h"
#include "nav_node.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }

static void usage(void)
{
    fprintf(stderr, "usage: nav [--no-gating] [--lockout N] [--gate P] [-v]\n");
}

int main(int argc, char **argv)
{
    nav_node_config_t cfg = nav_node_default_config();
    int verbose = 0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--no-gating")) cfg.gating = false;
        else if (!strcmp(argv[i], "--lockout") && i + 1 < argc) cfg.ekf.lockout_rejects = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--gate") && i + 1 < argc) cfg.ekf.gate_prob = atof(argv[++i]);
        else if (!strcmp(argv[i], "-v")) verbose = 1;
        else { usage(); return 2; }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    canbus_t bus;
    if (canbus_open(&bus) != 0) return 1;
    nav_node_t n;
    nav_node_init(&n, &cfg);
    fprintf(stderr, "nav: running (gating %s, lockout %d, gate %.4f)\n", cfg.gating ? "on" : "off",
            cfg.ekf.lockout_rejects, cfg.ekf.gate_prob);
    static const char *ST[] = { "NOMINAL", "DEGRADED", "UNAVAILABLE", "ALIGNING" };
    nav_status_t last = NAV_INIT;
    can_frame_t f, out[2 * NAV_OUT_MAX];
    while (!stop) {
        const int r = canbus_recv(&bus, &f, 20);
        if (r < 0) { perror("nav: recv"); break; }
        const int64_t now = clock_now_us();
        int k = r ? nav_node_on_frame(&n, &f, now, out) : 0;
        k += nav_node_on_idle(&n, now, out + k);
        for (int i = 0; i < k; ++i) canbus_send(&bus, &out[i]);
        const nav_status_t s = n.imu_lost ? NAV_UNAVAILABLE : n.status;
        if (verbose && s != last) {
            fprintf(stderr, "nav: t=%.2f status %s%s\n", n.t, ST[s], n.imu_lost ? " (IMU lost)" : "");
            last = s;
        }
    }
    canbus_close(&bus);
    fprintf(stderr, "nav: stopped\n");
    return 0;
}
