#include "cluster_core.h"
#include <math.h>
#include <string.h>

/* ---------- R1: debounce, per bit, sample based ----------
 * A bit changes only after DEBOUNCE_SAMPLES samples in a row disagree with the stable level.
 * One agreeing sample resets the count, so bounce never gets through. */
void debounce_init(debounce_t *d) { memset(d, 0, sizeof *d); }

uint8_t debounce_step(debounce_t *d, uint8_t raw)
{
    for (int b = 0; b < 6; ++b) {
        const uint8_t m = (uint8_t)(1u << b);
        if ((raw & m) == (d->stable & m)) {
            d->count[b] = 0;
        } else if (++d->count[b] >= DEBOUNCE_SAMPLES) {
            d->stable ^= m;
            d->count[b] = 0;
        }
    }
    return d->stable;
}

/* ---------- R4: blinker phase ---------- */
bool blink_lit(int64_t start_ms, int64_t now_ms)
{
    const int64_t ph = (now_ms - start_ms) % BLINK_PERIOD_MS;
    return ph >= 0 && ph < BLINK_ON_MS;
}

void core_init(cluster_core_t *c, int64_t now_ms)
{
    memset(c, 0, sizeof *c);
    c->now_ms = c->start_ms = now_ms;
    debounce_init(&c->deb);
    e2e_rx_init(&c->rx_inputs, 3, 1);       /* up to 2 lost frames tolerated, 1 frame to resync */
    e2e_rx_init(&c->rx_engine, 3, 1);
    e2e_rx_init(&c->rx_nav, 2, 0);
}

static void reset_inputs(cluster_core_t *c)
{
    debounce_init(&c->deb);
    c->hazard = false;
    c->ind = IND_NONE;
    c->blink_active = false;
}

static void on_inputs(cluster_core_t *c, const can_frame_t *f)
{
    const e2e_status_t s = e2e_check(&c->rx_inputs, (uint16_t)f->id, f->data);
    if (!e2e_usable(s)) {                    /* check first, decode only trusted frames */
        if (s != E2E_INITIAL) c->stats.inputs_rejected++;
        return;
    }
    msg_inputs_t m;
    msg_decode_inputs(f, &m);
    c->inputs_last_ms = c->now_ms;
    c->inputs_seen = true;
    if (c->input_fault) { c->input_fault = false; reset_inputs(c); }   /* start clean after R10 */

    const uint8_t prev = c->deb.stable;
    const uint8_t st = debounce_step(&c->deb, m.bits);
    /* R2: hazard toggles on the debounced rising edge */
    const bool hazard_before = c->hazard;
    if ((st & IN_HAZARD_BTN) && !(prev & IN_HAZARD_BTN)) c->hazard = !c->hazard;
    /* R3: both stalk contacts at once is implausible */
    indicator_t ind = IND_NONE;
    const bool l = (st & IN_STALK_LEFT) != 0, r = (st & IN_STALK_RIGHT) != 0;
    if (l && !r) ind = IND_LEFT;
    else if (r && !l) ind = IND_RIGHT;
    /* R4: restart the phase when what is blinking changes, so it always starts lit */
    const bool active_before = c->blink_active;
    const bool active = c->hazard || ind != IND_NONE;
    if (active && (!active_before || c->hazard != hazard_before || (!c->hazard && ind != c->ind)))
        c->blink_start_ms = c->now_ms;
    c->ind = ind;
    c->blink_active = active;
}

/* ---------- R6..R9: the ECU ---------- */
static void ecu_set_invalid(cluster_core_t *c)
{
    c->ecu_valid = false;
    c->ecu_good_run = 0;
    c->have_prev_speed = false;
}

static void on_engine(cluster_core_t *c, const can_frame_t *f)
{
    /* R6, step 1: E2E. A rejected frame does not refresh ecu_last_ms, so a stream of bad
     * frames ends in the same timeout (R7) as no frames at all. */
    const e2e_status_t s = e2e_check(&c->rx_engine, (uint16_t)f->id, f->data);
    if (s == E2E_BAD_CRC || s == E2E_BAD_COUNTER) { c->stats.ecu_rejected_crc++; return; }
    if (s == E2E_REPEATED || s == E2E_WRONG_SEQUENCE) {
        c->stats.ecu_rejected_seq++;
        if (s == E2E_WRONG_SEQUENCE) c->ecu_good_run = 0;
        return;
    }
    msg_engine_t m;
    msg_decode_engine(f, &m);
    /* R6, step 2: range */
    if (m.rpm > ECU_RPM_MAX || m.speed_kph > ECU_SPEED_MAX_KPH) {
        c->stats.ecu_rejected_range++;
        c->ecu_good_run = 0;
        return;
    }
    /* R6, step 3: rate, scaled by the number of frames since the last one we used */
    const int frames = c->rx_engine.last_delta ? c->rx_engine.last_delta : 1;
    if (c->have_prev_speed && fabs(m.speed_kph - c->prev_speed_kph) > ECU_SPEED_STEP_MAX_KPH * frames) {
        c->stats.ecu_rejected_rate++;
        c->ecu_good_run = 0;
        c->have_prev_speed = false;          /* re-anchor on the next frame */
        return;
    }
    c->prev_speed_kph = m.speed_kph;
    c->have_prev_speed = true;
    if (s == E2E_INITIAL) { c->ecu_good_run = 0; return; }   /* sequence not yet confirmed */
    c->ecu_last_ms = c->now_ms;
    if (!c->ecu_valid && ++c->ecu_good_run < ECU_RECOVER_FRAMES) return;   /* R8 */
    c->ecu_valid = true;
    c->ecu_seen = true;      /* has been valid once: from now on every loss is a fault (no start-up grace) */
    c->ecu_speed_kph = m.speed_kph;
    c->ecu_rpm = m.rpm;
    c->ecu_mil = m.mil;
}

/* ---------- R11, R12: the nav process ---------- */
static void on_nav(cluster_core_t *c, const can_frame_t *f)
{
    const e2e_status_t s = e2e_check(&c->rx_nav, (uint16_t)f->id, f->data);
    if (!e2e_usable(s)) { if (s != E2E_INITIAL) c->stats.nav_rejected++; return; }
    msg_decode_nav_state(f, &c->nav);
    c->nav_last_ms = c->now_ms;
    c->nav_seen = true;
}

void core_on_frame(cluster_core_t *c, const can_frame_t *f, int64_t now_ms)
{
    c->now_ms = now_ms;
    switch (f->id) {
    case CAN_ID_INPUTS: on_inputs(c, f); break;
    case CAN_ID_ENGINE: on_engine(c, f); break;
    case CAN_ID_NAV_STATE: on_nav(c, f); break;
    default: break;
    }
}

static void supervise(cluster_core_t *c)
{
    /* R7: no usable ECU frame for 100 ms */
    if (c->ecu_valid && c->now_ms - c->ecu_last_ms > ECU_TIMEOUT_MS) {
        ecu_set_invalid(c);
        c->stats.ecu_timeouts++;
    }
    /* R10: no usable INPUTS frame for 100 ms (or none at all since start) */
    const bool in_timeout = c->inputs_seen ? c->now_ms - c->inputs_last_ms > INPUT_TIMEOUT_MS
                                           : c->now_ms - c->start_ms > INPUT_TIMEOUT_MS;
    if (in_timeout && !c->input_fault) {
        c->input_fault = true;
        reset_inputs(c);
        c->stats.input_timeouts++;
    }
    /* R11: no usable NAV_STATE frame for 300 ms */
    if (c->nav_seen && c->now_ms - c->nav_last_ms > NAV_TIMEOUT_MS) {
        c->nav_seen = false;
        c->stats.nav_timeouts++;
    }
}

void core_outputs(const cluster_core_t *c, msg_cluster_a_t *a, msg_cluster_b_t *b)
{
    memset(a, 0, sizeof *a);
    memset(b, 0, sizeof *b);
    /* gauges: R7, R9, R13 */
    if (c->ecu_valid) {
        a->speed_dkph = (uint16_t)lround(c->ecu_speed_kph * 10.0);
        a->rpm = (uint16_t)lround(c->ecu_rpm);
        if (c->ecu_mil) a->lamps |= LAMP_CHECK_ENGINE;           /* R9: only from trusted data */
    } else {
        a->speed_dkph = CAN_U16_INVALID;                          /* R13: "--", never stale */
        a->rpm = CAN_U16_INVALID;
        if (c->ecu_seen || c->now_ms - c->start_ms > ECU_STARTUP_MS) a->lamps |= LAMP_ECU_FAULT;
    }
    /* lamps from inputs: R2..R5, R10 */
    if (c->input_fault) {
        a->lamps2 |= LAMP2_INPUT_FAULT;      /* we don't know the switch positions: all off */
    } else {
        const bool lit = c->blink_active && blink_lit(c->blink_start_ms, c->now_ms);
        if (lit && (c->hazard || c->ind == IND_LEFT)) a->lamps |= LAMP_LEFT;
        if (lit && (c->hazard || c->ind == IND_RIGHT)) a->lamps |= LAMP_RIGHT;
        if (c->hazard) a->lamps2 |= LAMP2_HAZARD;
        const uint8_t st = c->deb.stable;
        if (st & IN_DIP_SWITCH) a->lamps |= LAMP_DIP;
        if (((st & IN_DIP_SWITCH) && (st & IN_MAIN_BEAM)) || (st & IN_FLASH)) a->lamps |= LAMP_MAIN;
    }
    /* positioning: R11, R12 */
    b->heading_cdeg = CAN_U16_INVALID;
    b->pos_std_cm = CAN_U16_INVALID;
    if (!c->nav_seen) {
        b->text = POS_TXT_NAV_LOST;
        a->lamps |= LAMP_POS_RED;
        return;
    }
    switch (c->nav.status) {
    case NAV_NOMINAL: b->text = POS_TXT_OK; break;
    case NAV_DEGRADED: b->text = POS_TXT_DEGRADED; a->lamps |= LAMP_POS_AMBER; break;
    case NAV_INIT: b->text = POS_TXT_ALIGNING; a->lamps |= LAMP_POS_AMBER; break;
    default: b->text = POS_TXT_UNAVAILABLE; a->lamps |= LAMP_POS_RED; break;
    }
    if (c->nav.status == NAV_NOMINAL || c->nav.status == NAV_DEGRADED) {   /* heading only if trusted */
        long h = lround(c->nav.heading_deg * 100.0) % 36000;
        b->heading_cdeg = (uint16_t)(h < 0 ? h + 36000 : h);
        b->pos_std_cm = (uint16_t)lround(fmin(c->nav.pos_std_m * 100.0, 65534.0));
    }
}

int core_tick(cluster_core_t *c, int64_t now_ms, can_frame_t out[2])
{
    c->now_ms = now_ms;
    supervise(c);
    if (c->out_started && now_ms - c->last_out_ms < CORE_OUT_PERIOD_MS) return 0;
    /* keep a steady 50 ms grid; after a stall, restart the grid instead of sending a burst */
    c->last_out_ms = c->out_started ? c->last_out_ms + CORE_OUT_PERIOD_MS : now_ms;
    if (now_ms - c->last_out_ms >= CORE_OUT_PERIOD_MS) c->last_out_ms = now_ms;
    c->out_started = true;
    msg_cluster_a_t a;
    msg_cluster_b_t b;
    core_outputs(c, &a, &b);
    msg_encode_cluster_a(&out[0], &a);
    e2e_protect(&c->tx_a, (uint16_t)out[0].id, out[0].data);
    msg_encode_cluster_b(&out[1], &b);
    e2e_protect(&c->tx_b, (uint16_t)out[1].id, out[1].data);
    return 2;
}
