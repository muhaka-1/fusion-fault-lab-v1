/* cluster_core: the decision process. Runs a 10 ms cycle on CLOCK_MONOTONIC. */
#include "canbus.h"
#include "clock.h"
#include "cluster_core.h"
#include <signal.h>
#include <stdio.h>
#include <string.h>

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }

int main(int argc, char **argv)
{
    int verbose = argc > 1 && !strcmp(argv[1], "-v");
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    canbus_t bus;
    if (canbus_open(&bus) != 0) return 1;
    cluster_core_t c;
    int64_t next = clock_now_us();
    core_init(&c, next / 1000);
    fprintf(stderr, "cluster_core: running, %d ms cycle\n", CORE_TICK_MS);
    uint8_t last_lamps = 0xFF;
    while (!stop) {
        next += CORE_TICK_MS * 1000;
        /* drain the bus until the next cycle starts */
        for (;;) {
            int64_t left = next - clock_now_us();
            if (left <= 0) break;
            can_frame_t f;
            const int r = canbus_recv(&bus, &f, (int)((left + 999) / 1000));
            if (r < 0) { perror("cluster_core: recv"); stop = 1; break; }
            if (r == 1) core_on_frame(&c, &f, clock_now_us() / 1000);
        }
        const int64_t now = clock_now_us();
        if (now - next > 50000) next = now;            /* far behind (debugger, suspend): resync */
        can_frame_t out[2];
        if (core_tick(&c, now / 1000, out) == 2) {
            canbus_send(&bus, &out[0]);
            canbus_send(&bus, &out[1]);
            if (verbose && (out[0].data[6] & ~(LAMP_LEFT | LAMP_RIGHT)) != last_lamps) {
                last_lamps = out[0].data[6] & ~(LAMP_LEFT | LAMP_RIGHT);
                fprintf(stderr, "cluster_core: lamps 0x%02x lamps2 0x%02x\n", out[0].data[6], out[0].data[7]);
            }
        }
    }
    canbus_close(&bus);
    fprintf(stderr, "cluster_core: stopped (ECU rejected crc/seq/range/rate %u/%u/%u/%u, timeouts ecu/input/nav %u/%u/%u)\n",
            c.stats.ecu_rejected_crc, c.stats.ecu_rejected_seq, c.stats.ecu_rejected_range, c.stats.ecu_rejected_rate,
            c.stats.ecu_timeouts, c.stats.input_timeouts, c.stats.nav_timeouts);
    return 0;
}
