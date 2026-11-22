/* Offline harness: runs the C EKF + monitor over a CSV of sensor samples and prints the
 * estimate per step. Used by tests/test_parity.py to compare against the Python filter.
 *
 * stdin:  line 1   "x0,y0,psi0,v0,gating,dt"
 *         then     "t,gyro,acc,wheel,gx,gy"  (wheel/gx/gy empty when absent)
 * stdout: "t,x,y,psi,v,b,pos_std,status,gnss_ok,wheel_ok,gnss_nis,wheel_nis,recoveries"
 */
#include "nav_ekf.h"
#include "nav_monitor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *ST[] = { "NOMINAL", "DEGRADED", "UNAVAILABLE", "INIT" };

static int field(char **p, double *out)
{
    char *s = *p, *e = strchr(s, ',');
    if (e) *e = '\0';
    char *nl = strpbrk(s, "\r\n");
    if (nl) *nl = '\0';
    const int has = *s != '\0';
    if (has) *out = strtod(s, NULL);
    *p = e ? e + 1 : s + strlen(s);
    return has;
}

int main(void)
{
    char line[512];
    double h[6];
    if (!fgets(line, sizeof line, stdin)) return 1;
    char *p = line;
    for (int i = 0; i < 6; ++i) if (!field(&p, &h[i])) return 1;
    ekf_t e;
    monitor_t m;
    ekf_init(&e, h, NULL, h[4] != 0.0);
    monitor_init(&m, NULL);
    const double dt = h[5];
    printf("t,x,y,psi,v,b,pos_std,status,gnss_ok,wheel_ok,gnss_nis,wheel_nis,recoveries\n");
    for (long i = 0; fgets(line, sizeof line, stdin); ++i) {
        double t = 0, g = 0, a = 0, w = 0, gx = 0, gy = 0;
        p = line;
        field(&p, &t); field(&p, &g); field(&p, &a);
        const int hw = field(&p, &w), hx = field(&p, &gx), hy = field(&p, &gy);
        if (i > 0) ekf_predict(&e, g, a, dt);
        int wok = -1, gok = -1;
        double wn = 0, gn = 0;
        if (hw) { ekf_update_t u = ekf_update_wheel(&e, w); monitor_on_update(&m, MON_WHEEL, t, u.nis, u.accepted); wok = u.accepted; wn = u.nis; }
        if (hx && hy) { ekf_update_t u = ekf_update_gnss(&e, gx, gy); monitor_on_update(&m, MON_GNSS, t, u.nis, u.accepted); gok = u.accepted; gn = u.nis; }
        printf("%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%s,%d,%d,%.17g,%.17g,%d\n", t, e.x[0], e.x[1], e.x[2], e.x[3], e.x[4],
               ekf_pos_std(&e), ST[monitor_status(&m, t, ekf_pos_std(&e))], gok, wok, gn, wn, e.recoveries);
    }
    return 0;
}
