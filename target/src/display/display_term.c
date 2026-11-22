/* display_term: draws the cluster in a terminal (QNX console, ssh, Linux).
 * It makes no decisions about vehicle state: it draws CLUSTER_A / CLUSTER_B exactly as the
 * core sent them. Its only rule is about itself: if no valid frame from the core arrives
 * for 250 ms it blanks and says so, rather than freezing on the last picture.
 *
 *   display_term          full-screen ANSI drawing at 20 Hz
 *   display_term --plain  one text line per change (logs, tests, serial consoles)
 */
#include "canbus.h"
#include "clock.h"
#include "e2e.h"
#include <signal.h>
#include <stdio.h>
#include <string.h>

#define CORE_LOST_MS 250

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }

static const char *POS[] = { "OK", "DEGRADED", "UNAVAILABLE", "ALIGNING", "NAV DATA LOST" };

static void fmt_u16(char *buf, size_t n, uint16_t v, double scale, const char *f)
{
    if (v == CAN_U16_INVALID) snprintf(buf, n, "--");
    else snprintf(buf, n, f, v * scale);
}

static void line(char *out, size_t n, bool lost, const msg_cluster_a_t *a, const msg_cluster_b_t *b)
{
    if (lost) { snprintf(out, n, "NO SIGNAL FROM CORE"); return; }
    char sp[16], rpm[16], hd[16];
    fmt_u16(sp, sizeof sp, a->speed_dkph, 0.1, "%.0f");
    fmt_u16(rpm, sizeof rpm, a->rpm, 1.0, "%.0f");
    fmt_u16(hd, sizeof hd, b->heading_cdeg, 0.01, "%.0f");
    snprintf(out, n, "speed=%s rpm=%s left=%d right=%d hazard=%d dip=%d main=%d check_engine=%d ecu_data=%s input=%s pos=%s heading=%s",
             sp, rpm, !!(a->lamps & LAMP_LEFT), !!(a->lamps & LAMP_RIGHT), !!(a->lamps2 & LAMP2_HAZARD),
             !!(a->lamps & LAMP_DIP), !!(a->lamps & LAMP_MAIN), !!(a->lamps & LAMP_CHECK_ENGINE),
             (a->lamps & LAMP_ECU_FAULT) ? "FAULT" : "ok", (a->lamps2 & LAMP2_INPUT_FAULT) ? "FAULT" : "ok",
             b->text <= POS_TXT_NAV_LOST ? POS[b->text] : "?", hd);
}

static void lamp(const char *on_color, bool on, const char *label)
{
    if (on) printf("\x1b[%sm %s \x1b[0m ", on_color, label);
    else printf("\x1b[2m %s \x1b[0m ", label);
}

static void draw(bool lost, const msg_cluster_a_t *a, const msg_cluster_b_t *b)
{
    printf("\x1b[H\x1b[2J");
    printf("  fusion-fault-lab cluster                 (display_term: draws what the core sends)\n\n");
    if (lost) {
        printf("\n\n          \x1b[1;31mNO SIGNAL FROM CORE\x1b[0m\n\n          gauges blanked, no stale values shown\n");
        fflush(stdout);
        return;
    }
    char sp[16], rpm[16], hd[16], sd[16];
    fmt_u16(sp, sizeof sp, a->speed_dkph, 0.1, "%5.0f");
    fmt_u16(rpm, sizeof rpm, a->rpm, 1.0, "%5.0f");
    fmt_u16(hd, sizeof hd, b->heading_cdeg, 0.01, "%3.0f");
    fmt_u16(sd, sizeof sd, b->pos_std_cm, 0.01, "%.2f m");
    const int barw = 40;
    int sbar = a->speed_dkph == CAN_U16_INVALID ? 0 : (int)(a->speed_dkph / 2000.0 * barw);
    int rbar = a->rpm == CAN_U16_INVALID ? 0 : (int)(a->rpm / 7000.0 * barw);
    if (sbar > barw) sbar = barw;
    if (rbar > barw) rbar = barw;
    printf("   ");
    lamp("1;30;42", a->lamps & LAMP_LEFT, "<<");
    printf("%*s", 46, "");
    lamp("1;30;42", a->lamps & LAMP_RIGHT, ">>");
    printf("\n\n   SPEED %6s km/h  [", sp);
    for (int i = 0; i < barw; ++i) putchar(i < sbar ? '#' : '.');
    printf("]\n   RPM   %6s       [", rpm);
    for (int i = 0; i < barw; ++i) putchar(i < rbar ? '#' : '.');
    printf("]\n\n   ");
    lamp("1;30;42", a->lamps & LAMP_DIP, "DIPPED");
    lamp("1;37;44", a->lamps & LAMP_MAIN, "MAIN BEAM");
    lamp("1;37;41", a->lamps2 & LAMP2_HAZARD, "HAZARD");
    lamp("1;30;43", a->lamps & LAMP_CHECK_ENGINE, "CHECK ENGINE");
    printf("\n   ");
    lamp("1;37;41", a->lamps & LAMP_ECU_FAULT, "ECU DATA");
    lamp("1;37;41", a->lamps2 & LAMP2_INPUT_FAULT, "INPUTS");
    const char *pc = (a->lamps & LAMP_POS_RED) ? "1;37;41" : "1;30;43";
    lamp(pc, a->lamps & (LAMP_POS_RED | LAMP_POS_AMBER), "POSITIONING");
    printf("\n\n   heading %s deg   position %s   %s\n", hd, b->text <= POS_TXT_NAV_LOST ? POS[b->text] : "?", sd);
    fflush(stdout);
}

int main(int argc, char **argv)
{
    const bool plain = argc > 1 && !strcmp(argv[1], "--plain");
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    canbus_t bus;
    if (canbus_open(&bus) != 0) return 1;
    e2e_rx_t rxa, rxb;
    e2e_rx_init(&rxa, 2, 0);
    e2e_rx_init(&rxb, 2, 0);
    msg_cluster_a_t a;
    msg_cluster_b_t b;
    memset(&a, 0, sizeof a);
    memset(&b, 0, sizeof b);
    int64_t last_ok = 0, last_ok_b = 0, last_draw = 0;
    char prev[512] = "", cur[512];
    if (!plain) printf("\x1b[?25l");
    while (!stop) {
        can_frame_t f;
        const int r = canbus_recv(&bus, &f, 20);
        if (r < 0) break;
        const int64_t now = clock_now_us() / 1000;
        if (r == 1 && f.id == CAN_ID_CLUSTER_A && e2e_usable(e2e_check(&rxa, (uint16_t)f.id, f.data))) {
            msg_decode_cluster_a(&f, &a);
            last_ok = now;
        } else if (r == 1 && f.id == CAN_ID_CLUSTER_B && e2e_usable(e2e_check(&rxb, (uint16_t)f.id, f.data))) {
            msg_decode_cluster_b(&f, &b);
            last_ok_b = now;
        }
        /* draw nothing until both frames are fresh: never a half-initialised picture */
        const bool lost = !last_ok || !last_ok_b || now - last_ok > CORE_LOST_MS || now - last_ok_b > CORE_LOST_MS;
        if (plain) {
            line(cur, sizeof cur, lost, &a, &b);
            if (strcmp(cur, prev)) {
                printf("%.3f %s\n", (double)now / 1000.0, cur);
                fflush(stdout);
                memcpy(prev, cur, sizeof cur);
            }
        } else if (now - last_draw >= 50) {
            last_draw = now;
            draw(lost, &a, &b);
        }
    }
    if (!plain) printf("\x1b[0m\x1b[?25h\n");
    canbus_close(&bus);
    return 0;
}
