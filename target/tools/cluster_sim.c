/* cluster_sim: drive the real cluster core with a scripted bench and draw a timing diagram.
 *
 *   cluster_sim                       built-in demo (M10: ECU and nav faults)
 *   cluster_sim --inputs              built-in demo of M9 (stalks, hazard, beams)
 *   cluster_sim STEP STEP ...         your own script
 *
 * A STEP is TIME_MS:WHAT. WHAT is either switches or a list of settings.
 *   switches   any mix of L stalk left, R stalk right, H hazard, D dipped beam, M main beam,
 *              F flash, - nothing, X input board silent, ~ prefix: contacts bounce for 20 ms
 *   settings   comma-separated, e.g. 1000:ecu=freeze  or  2000:speed=80,rpm=2500
 *              ecu=ok|off|corrupt|freeze     ok: normal, off: no frames, corrupt: bad CRC,
 *                                            freeze: alive counter stuck (ECU software hung)
 *              speed=KPH  rpm=RPM  mil=0|1   what the ECU sends (it starts at 50 km/h, 1500 rpm)
 *              nav=nominal|degraded|unavailable|aligning|off      (starts nominal, heading 90)
 * The bench sends INPUTS every 10 ms, ENGINE every 20 ms, NAV_STATE every 100 ms, like the car.
 * One column is one 50 ms output frame of the core.
 */
#include "cluster_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_EVENTS 64
#define MAX_COLS 2000
#define WIDTH 80
#define ROWS 19

typedef enum { ECU_OK, ECU_OFF, ECU_CORRUPT, ECU_FREEZE } ecu_mode_t;
typedef struct {
    int64_t t;
    bool is_switch;
    uint8_t bits; bool silent, bounce;                    /* switches */
    int ecu; double speed, rpm; int mil; int nav;         /* settings, -1 / <0 = unchanged */
    char text[64];
} event_t;

static const char *DEMO_M10[] = { "0:-", "1000:ecu=off", "1300:ecu=ok", "2200:ecu=corrupt", "2500:ecu=ok",
    "3300:ecu=freeze", "3600:ecu=ok", "4400:rpm=12000", "4600:rpm=1500", "5200:speed=150", "5220:speed=50",
    "5600:mil=1", "6000:nav=degraded", "6400:nav=unavailable", "6800:nav=off", "7400:nav=nominal",
    "7800:ecu=off,mil=0", "8200:ecu=ok" };
static const char *DEMO_M9[] = { "0:-", "200:L", "1500:-", "1800:~R", "2600:-", "3000:H", "3100:-",
    "4300:LR", "4800:-", "5000:H", "5100:-", "5400:D", "5900:DM", "6400:F", "6600:-", "6800:DX", "7300:D" };

static int parse_settings(char *s, event_t *e)
{
    for (char *tok = strtok(s, ","); tok; tok = strtok(NULL, ",")) {
        char *v = strchr(tok, '=');
        if (!v) return -1;
        *v++ = 0;
        if (!strcmp(tok, "ecu")) {
            if (!strcmp(v, "ok")) e->ecu = ECU_OK;
            else if (!strcmp(v, "off")) e->ecu = ECU_OFF;
            else if (!strcmp(v, "corrupt")) e->ecu = ECU_CORRUPT;
            else if (!strcmp(v, "freeze")) e->ecu = ECU_FREEZE;
            else return -1;
        } else if (!strcmp(tok, "speed")) e->speed = atof(v);
        else if (!strcmp(tok, "rpm")) e->rpm = atof(v);
        else if (!strcmp(tok, "mil")) e->mil = atoi(v) != 0;
        else if (!strcmp(tok, "nav")) {
            if (!strcmp(v, "nominal")) e->nav = NAV_NOMINAL;
            else if (!strcmp(v, "degraded")) e->nav = NAV_DEGRADED;
            else if (!strcmp(v, "unavailable")) e->nav = NAV_UNAVAILABLE;
            else if (!strcmp(v, "aligning")) e->nav = NAV_INIT;
            else if (!strcmp(v, "off")) e->nav = 99;
            else return -1;
        } else return -1;
    }
    return 0;
}

static int parse(const char *s, event_t *e)
{
    memset(e, 0, sizeof *e);
    e->ecu = -1; e->speed = -1; e->rpm = -1; e->mil = -1; e->nav = -1;
    snprintf(e->text, sizeof e->text, "%s", s);
    char buf[64];
    snprintf(buf, sizeof buf, "%s", s);
    char *end;
    e->t = strtoll(buf, &end, 10);
    if (*end != ':') return -1;
    char *what = end + 1;
    if (strchr(what, '=')) return parse_settings(what, e);
    e->is_switch = true;
    for (const char *p = what; *p; ++p) {
        switch (*p) {
        case 'L': e->bits |= IN_STALK_LEFT; break;
        case 'R': e->bits |= IN_STALK_RIGHT; break;
        case 'H': e->bits |= IN_HAZARD_BTN; break;
        case 'D': e->bits |= IN_DIP_SWITCH; break;
        case 'M': e->bits |= IN_MAIN_BEAM; break;
        case 'F': e->bits |= IN_FLASH; break;
        case 'X': e->silent = true; break;
        case '~': e->bounce = true; break;
        case '-': break;
        default: return -1;
        }
    }
    return 0;
}

static const char *POS[] = { "OK", "DEGRADED", "UNAVAILABLE", "ALIGNING", "NAV DATA LOST" };

static void fmt(char *b, size_t n, uint16_t v, double scale)
{
    if (v == CAN_U16_INVALID) snprintf(b, n, "--");
    else snprintf(b, n, "%.0f", v * scale);
}

int main(int argc, char **argv)
{
    event_t ev[MAX_EVENTS];
    int n = 0;
    const char **src = DEMO_M10;
    int count = (int)(sizeof DEMO_M10 / sizeof DEMO_M10[0]);
    int first = 0;
    if (argc > 1 && !strcmp(argv[1], "--inputs")) { src = DEMO_M9; count = (int)(sizeof DEMO_M9 / sizeof DEMO_M9[0]); }
    else if (argc > 1) { src = (const char **)argv; first = 1; count = argc; }
    for (int i = first; i < count && n < MAX_EVENTS; ++i) {
        if (parse(src[i], &ev[n]) != 0) { fprintf(stderr, "bad step '%s' (see the comment at the top of cluster_sim.c)\n", src[i]); return 2; }
        ++n;
    }
    if (n == 0) return 2;
    bool uses_switches = false;
    for (int i = 0; i < n; ++i) if (ev[i].is_switch && (ev[i].bits || ev[i].silent)) uses_switches = true;
    const int64_t t_end = ev[n - 1].t + 1000;

    cluster_core_t c;
    core_init(&c, 0);
    e2e_tx_t ti = { 0 }, te = { 0 }, tn = { 0 };
    static char row[ROWS][MAX_COLS + 1];
    int cols = 0, k = 0;
    uint8_t level = 0, before = 0;
    bool silent = false, bounce = false;
    int64_t changed_at = 0;
    ecu_mode_t ecu = ECU_OK;
    msg_engine_t eng = { false, true, 1500.0, 50.0 };
    int nav = NAV_NOMINAL;
    char log[8192] = "";
    size_t logn = 0;
    char last_sp[16] = "?", last_rpm[16] = "?", last_hd[16] = "?";
    int last_pos = -1, last_mil = -1;
    for (int64_t t = 0; t <= t_end && cols < MAX_COLS; t += CORE_TICK_MS) {
        while (k < n && ev[k].t <= t) {
            const event_t *e = &ev[k++];
            if (e->is_switch) {
                before = level; level = e->bits; silent = e->silent; bounce = e->bounce; changed_at = t;
            } else {
                if (e->ecu >= 0) ecu = (ecu_mode_t)e->ecu;
                if (e->speed >= 0) eng.speed_kph = e->speed;
                if (e->rpm >= 0) eng.rpm = e->rpm;
                if (e->mil >= 0) eng.mil = e->mil;
                if (e->nav >= 0) nav = e->nav;
            }
            if (logn < sizeof log - 100)
                logn += (size_t)snprintf(log + logn, sizeof log - logn, "  %5.2f s  bench: %s\n", (double)t / 1000.0, strchr(e->text, ':') + 1);
        }
        can_frame_t f;
        uint8_t raw = level;
        if (bounce && t - changed_at < 20 && ((t - changed_at) / 10) % 2 == 1) raw = before;
        if (!silent) {
            msg_inputs_t m = { raw };
            msg_encode_inputs(&f, &m);
            e2e_protect(&ti, (uint16_t)f.id, f.data);
            core_on_frame(&c, &f, t);
        }
        if (t % 20 == 0 && ecu != ECU_OFF) {
            msg_encode_engine(&f, &eng);
            const uint8_t ctr = te.counter;
            e2e_protect(&te, (uint16_t)f.id, f.data);
            if (ecu == ECU_FREEZE) {                   /* CAN controller resends, counter stuck */
                te.counter = ctr;
                f.data[1] = (uint8_t)((f.data[1] & 0xF0) | ctr);
                f.data[0] = e2e_crc8_frame((uint16_t)f.id, f.data);
            }
            if (ecu == ECU_CORRUPT) f.data[4] ^= 0x01;
            core_on_frame(&c, &f, t);
        }
        if (t % 100 == 0 && nav != 99) {
            msg_nav_state_t ns = { (nav_status_t)nav, 0, 90.0, 0.8, 0.2 };
            msg_encode_nav_state(&f, &ns);
            e2e_protect(&tn, (uint16_t)f.id, f.data);
            core_on_frame(&c, &f, t);
        }
        can_frame_t out[2];
        if (core_tick(&c, t, out) == 2) {
            msg_cluster_a_t a;
            msg_cluster_b_t b;
            msg_decode_cluster_a(&out[0], &a);
            msg_decode_cluster_b(&out[1], &b);
            const uint8_t ins[6] = { IN_STALK_LEFT, IN_STALK_RIGHT, IN_HAZARD_BTN, IN_DIP_SWITCH, IN_MAIN_BEAM, IN_FLASH };
            for (int r = 0; r < 6; ++r) row[r][cols] = silent ? 'x' : ((level & ins[r]) ? '#' : '.');
            row[6][cols] = "#xcf"[ecu];
            row[7][cols] = nav == 99 ? 'x' : "NDUA"[nav];
            row[8][cols] = (a.lamps & LAMP_LEFT) ? '#' : '.';
            row[9][cols] = (a.lamps & LAMP_RIGHT) ? '#' : '.';
            row[10][cols] = (a.lamps2 & LAMP2_HAZARD) ? '#' : '.';
            row[11][cols] = (a.lamps & LAMP_DIP) ? '#' : '.';
            row[12][cols] = (a.lamps & LAMP_MAIN) ? '#' : '.';
            row[13][cols] = (a.lamps2 & LAMP2_INPUT_FAULT) ? '#' : '.';
            row[14][cols] = a.speed_dkph == CAN_U16_INVALID ? '-' : '#';
            row[15][cols] = (a.lamps & LAMP_ECU_FAULT) ? '#' : '.';
            row[16][cols] = (a.lamps & LAMP_CHECK_ENGINE) ? '#' : '.';
            row[17][cols] = (a.lamps & LAMP_POS_RED) ? 'R' : ((a.lamps & LAMP_POS_AMBER) ? 'A' : '.');
            row[18][cols] = b.heading_cdeg == CAN_U16_INVALID ? '-' : '#';
            char sp[16], rpm[16], hd[16];
            fmt(sp, sizeof sp, a.speed_dkph, 0.1);
            fmt(rpm, sizeof rpm, a.rpm, 1.0);
            fmt(hd, sizeof hd, b.heading_cdeg, 0.01);
            const int mil = (a.lamps & LAMP_CHECK_ENGINE) != 0;
            if ((strcmp(sp, last_sp) || strcmp(rpm, last_rpm) || strcmp(hd, last_hd) || (int)b.text != last_pos ||
                 mil != last_mil) && logn < sizeof log - 140) {
                logn += (size_t)snprintf(log + logn, sizeof log - logn,
                                         "  %5.2f s  cluster: speed %s km/h, rpm %s, ECU DATA %s, CHECK ENG %s, POSITIONING %s, heading %s\n",
                                         (double)t / 1000.0, sp, rpm, (a.lamps & LAMP_ECU_FAULT) ? "RED" : "ok", mil ? "ON" : "off",
                                         b.text <= POS_TXT_NAV_LOST ? POS[b.text] : "?", hd);
                last_mil = mil;
                snprintf(last_sp, sizeof last_sp, "%s", sp);
                snprintf(last_rpm, sizeof last_rpm, "%s", rpm);
                snprintf(last_hd, sizeof last_hd, "%s", hd);
                last_pos = (int)b.text;
            }
            ++cols;
        }
    }
    static const char *name[ROWS] = {
        "in  stalk left ", "in  stalk right", "in  hazard btn ", "in  dipped sw  ", "in  main beam  ", "in  flash      ",
        "bus ECU frames ", "bus nav status ",
        "OUT << left    ", "OUT >> right   ", "OUT HAZARD     ", "OUT DIPPED     ", "OUT MAIN BEAM  ", "OUT INPUT FAULT",
        "OUT speed/rpm  ", "OUT ECU DATA   ", "OUT CHECK ENG  ", "OUT POSITIONING", "OUT heading    " };
    const bool lamps_rows = uses_switches;
    printf("one column = %d ms.  # on, . off, - shows \"--\", x no frames\n", CORE_OUT_PERIOD_MS);
    printf("bus ECU frames: # ok, x none, c corrupt, f frozen counter.  nav: N nominal, D degraded, U unavailable, A aligning\n");
    printf("POSITIONING lamp: A amber, R red\n");
    for (int c0 = 0; c0 < cols; c0 += WIDTH) {
        const int w = cols - c0 < WIDTH ? cols - c0 : WIDTH;
        printf("\n%-16s", "time [s]");
        for (int i = 0; i < w; i += 10) printf("%-10.1f", (c0 + i) * CORE_OUT_PERIOD_MS / 1000.0);
        printf("\n");
        for (int r = 0; r < ROWS; ++r) {
            if (!lamps_rows && (r < 6 || (r >= 8 && r <= 13))) continue;
            if (r == 8 || (r == 14 && !lamps_rows))
                printf("%-16s%.*s\n", "", w, "--------------------------------------------------------------------------------");
            printf("%s %.*s\n", name[r], w, &row[r][c0]);
        }
    }
    printf("\nwhat happened:\n%s", log);
    printf("\ncore stats: ECU rejected crc %u, sequence %u, range %u, rate %u; timeouts ECU %u, input %u, nav %u\n",
           c.stats.ecu_rejected_crc, c.stats.ecu_rejected_seq, c.stats.ecu_rejected_range, c.stats.ecu_rejected_rate,
           c.stats.ecu_timeouts, c.stats.input_timeouts, c.stats.nav_timeouts);
    return 0;
}
