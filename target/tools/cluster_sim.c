/* cluster_sim: drive the real cluster core with a scripted input board and draw a timing diagram.
 *
 *   cluster_sim                       run the built-in demo script
 *   cluster_sim 0:- 200:L 1500:-      your own script: TIME_MS:SWITCHES ...
 *
 * SWITCHES is any mix of   L stalk left   R stalk right   H hazard button   D dipped beam
 *                          M main beam    F flash         -  nothing pressed
 *                          X  input board goes silent (no frames at all)
 *                          ~  (prefix) the contacts bounce for 20 ms before settling
 * One column is one 50 ms output frame of the core. # = on, . = off, x = no frames.
 */
#include "cluster_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_EVENTS 64
#define MAX_COLS 2000
#define WIDTH 80

typedef struct { int64_t t; uint8_t bits; bool silent, bounce; } event_t;

static const char *DEMO[] = { "0:-", "200:L", "1500:-", "1800:~R", "2600:-", "3000:H", "3100:-",
                              "4300:LR", "4800:-", "5000:H", "5100:-", "5400:D", "5900:DM", "6400:F",
                              "6600:-", "6800:DX", "7300:D" };

static int parse(const char *s, event_t *e)
{
    char *end;
    e->t = strtoll(s, &end, 10);
    if (*end != ':') return -1;
    e->bits = 0; e->silent = false; e->bounce = false;
    for (const char *p = end + 1; *p; ++p) {
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

int main(int argc, char **argv)
{
    event_t ev[MAX_EVENTS];
    int n = 0;
    const int use_demo = argc < 2;
    const int count = use_demo ? (int)(sizeof DEMO / sizeof DEMO[0]) : argc - 1;
    for (int i = 0; i < count && n < MAX_EVENTS; ++i) {
        const char *s = use_demo ? DEMO[i] : argv[i + 1];
        if (parse(s, &ev[n]) != 0) { fprintf(stderr, "bad step '%s' (want TIME_MS:LRHDMF-X~)\n", s); return 2; }
        ++n;
    }
    if (n == 0) return 2;
    const int64_t t_end = ev[n - 1].t + 1000;

    cluster_core_t c;
    core_init(&c, 0);
    e2e_tx_t tx = { 0 };
    static char row[12][MAX_COLS + 1];
    int cols = 0, k = 0;
    uint8_t level = 0, before = 0;
    bool silent = false, bounce = false;
    int64_t changed_at = 0;
    for (int64_t t = 0; t <= t_end && cols < MAX_COLS; t += CORE_TICK_MS) {
        while (k < n && ev[k].t <= t) {
            before = level;
            level = ev[k].bits; silent = ev[k].silent; bounce = ev[k].bounce; changed_at = t;
            ++k;
        }
        uint8_t raw = level;
        if (bounce && t - changed_at < 20 && ((t - changed_at) / 10) % 2 == 1) raw = before;
        if (!silent) {
            can_frame_t f;
            msg_inputs_t m = { raw };
            msg_encode_inputs(&f, &m);
            e2e_protect(&tx, (uint16_t)f.id, f.data);
            core_on_frame(&c, &f, t);
        }
        can_frame_t out[2];
        if (core_tick(&c, t, out) == 2) {
            msg_cluster_a_t a;
            msg_decode_cluster_a(&out[0], &a);
            const uint8_t ins[6] = { IN_STALK_LEFT, IN_STALK_RIGHT, IN_HAZARD_BTN, IN_DIP_SWITCH, IN_MAIN_BEAM, IN_FLASH };
            for (int r = 0; r < 6; ++r) row[r][cols] = silent ? 'x' : ((level & ins[r]) ? '#' : '.');
            row[6][cols] = (a.lamps & LAMP_LEFT) ? '#' : '.';
            row[7][cols] = (a.lamps & LAMP_RIGHT) ? '#' : '.';
            row[8][cols] = (a.lamps2 & LAMP2_HAZARD) ? '#' : '.';
            row[9][cols] = (a.lamps & LAMP_DIP) ? '#' : '.';
            row[10][cols] = (a.lamps & LAMP_MAIN) ? '#' : '.';
            row[11][cols] = (a.lamps2 & LAMP2_INPUT_FAULT) ? '#' : '.';
            ++cols;
        }
    }
    static const char *name[12] = { "in  stalk left ", "in  stalk right", "in  hazard btn ", "in  dipped sw  ",
                                    "in  main beam  ", "in  flash      ", "OUT << left    ", "OUT >> right   ",
                                    "OUT HAZARD     ", "OUT DIPPED     ", "OUT MAIN BEAM  ", "OUT INPUT FAULT" };
    printf("script:");
    for (int i = 0; i < n; ++i) {
        printf(" %lld:", (long long)ev[i].t);
        if (ev[i].bounce) putchar('~');
        if (ev[i].silent) putchar('X');
        const char *L = "LRHDMF";
        for (int b = 0; b < 6; ++b) if (ev[i].bits & (1u << b)) putchar(L[b]);
        if (!ev[i].bits && !ev[i].silent) putchar('-');
    }
    printf("\none column = %d ms, # on, . off, x no frames from the input board\n", CORE_OUT_PERIOD_MS);
    for (int c0 = 0; c0 < cols; c0 += WIDTH) {
        const int w = cols - c0 < WIDTH ? cols - c0 : WIDTH;
        printf("\n%-16s", "time [s]");
        for (int i = 0; i < w; i += 10) printf("%-10.1f", (c0 + i) * CORE_OUT_PERIOD_MS / 1000.0);
        printf("\n");
        for (int r = 0; r < 12; ++r) {
            if (r == 6) printf("%-16s%.*s\n", "", w, "--------------------------------------------------------------------------------");
            printf("%s %.*s\n", name[r], w, &row[r][c0]);
        }
    }
    printf("\nE2E-rejected input frames: %u, input timeouts: %u\n", c.stats.inputs_rejected, c.stats.input_timeouts);
    return 0;
}
