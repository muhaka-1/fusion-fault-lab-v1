/* Cluster core rules: inputs R1-R5, R10 (M9); ECU R6-R9, nav R11-R12, R13 (M10).
 * Every test drives the real core through a fake bench with its own clock: no sleeping,
 * no bus, fully repeatable. */
#include "test.h"
#include "cluster_core.h"
#include <string.h>

/* A fake bench: input board every 10 ms, ECU every 20 ms, nav every 100 ms, core ticks every 10 ms. */
typedef struct {
    cluster_core_t c;
    int64_t t;
    e2e_tx_t ti, te, tn;
    uint8_t inputs;
    bool send_inputs, send_ecu, send_nav;
    msg_engine_t ecu;
    msg_nav_state_t nav;
    int corrupt_ecu;       /* corrupt the next N ECU frames */
    int corrupt_inputs;    /* corrupt the next N INPUTS frames */
    bool freeze_ecu_counter;
    msg_cluster_a_t a;
    msg_cluster_b_t b;
    int outputs;
} bench_t;

static void bench_init(bench_t *b)
{
    memset(b, 0, sizeof *b);
    b->t = 1000;
    core_init(&b->c, b->t);
    b->send_inputs = b->send_ecu = b->send_nav = true;
    b->ecu.running = true;
    b->ecu.rpm = 1500;
    b->ecu.speed_kph = 50;
    b->nav.status = NAV_NOMINAL;
    b->nav.heading_deg = 90;
    b->nav.pos_std_m = 0.5;
}

static void step(bench_t *b, int ms)
{
    for (int i = 0; i < ms; i += 10) {
        b->t += 10;
        can_frame_t f, out[2];
        if (b->send_inputs) {
            msg_inputs_t m = { b->inputs };
            msg_encode_inputs(&f, &m); e2e_protect(&b->ti, (uint16_t)f.id, f.data);
            if (b->corrupt_inputs > 0) { f.data[2] ^= 0x01; b->corrupt_inputs--; }
            core_on_frame(&b->c, &f, b->t);
        }
        if (b->send_ecu && b->t % 20 == 0) {
            msg_encode_engine(&f, &b->ecu);
            const uint8_t ctr = b->te.counter;
            e2e_protect(&b->te, (uint16_t)f.id, f.data);
            if (b->freeze_ecu_counter) {
                b->te.counter = ctr;
                f.data[1] = (uint8_t)((f.data[1] & 0xF0) | ctr);
                f.data[0] = e2e_crc8_frame((uint16_t)f.id, f.data);
            }
            if (b->corrupt_ecu > 0) { f.data[4] ^= 0x01; b->corrupt_ecu--; }
            core_on_frame(&b->c, &f, b->t);
        }
        if (b->send_nav && b->t % 100 == 0) {
            msg_encode_nav_state(&f, &b->nav); e2e_protect(&b->tn, (uint16_t)f.id, f.data);
            core_on_frame(&b->c, &f, b->t);
        }
        if (core_tick(&b->c, b->t, out) == 2) {
            msg_decode_cluster_a(&out[0], &b->a);
            msg_decode_cluster_b(&out[1], &b->b);
            b->outputs++;
        }
    }
}

static bool lamp(const bench_t *b, uint8_t m) { return (b->a.lamps & m) != 0; }

/* ---- R1 debounce ---- */
static void debounce_needs_three_stable_samples(void)
{
    debounce_t d;
    debounce_init(&d);
    CHECK_EQ(debounce_step(&d, IN_STALK_LEFT), 0);
    CHECK_EQ(debounce_step(&d, IN_STALK_LEFT), 0);
    CHECK_EQ(debounce_step(&d, IN_STALK_LEFT), IN_STALK_LEFT);
}

static void debounce_ignores_bounce(void)
{
    debounce_t d;
    debounce_init(&d);
    const uint8_t bounce[] = { 1, 0, 1, 1, 0, 1, 0, 0, 1 };
    for (unsigned i = 0; i < sizeof bounce; ++i) CHECK_EQ(debounce_step(&d, bounce[i]), 0);
    CHECK_EQ(debounce_step(&d, 1), 0);
    CHECK_EQ(debounce_step(&d, 1), 1);
}

static void debounce_bits_are_independent(void)
{
    debounce_t d;
    debounce_init(&d);
    for (int i = 0; i < 3; ++i) debounce_step(&d, IN_DIP_SWITCH);
    debounce_step(&d, IN_DIP_SWITCH | IN_FLASH);
    CHECK_EQ(d.stable, IN_DIP_SWITCH);
    debounce_step(&d, IN_FLASH);                 /* dip released, but only once */
    CHECK_EQ(d.stable, IN_DIP_SWITCH);
}

/* ---- R4 blinker ---- */
static void blink_90_per_minute_half_duty(void)
{
    int lit = 0;
    for (int ms = 0; ms < 6670; ++ms) lit += blink_lit(0, ms);
    CHECK_EQ(lit, 3330);                         /* 10 periods of 333 ms on */
    CHECK(blink_lit(0, 0));
    CHECK(!blink_lit(0, 333));
    CHECK(blink_lit(0, 667));
}

static void left_indicator_blinks(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.inputs = IN_STALK_LEFT;
    step(&b, 30);
    int on = 0, off = 0;
    for (int i = 0; i < 40; ++i) {
        step(&b, 50);
        on += lamp(&b, LAMP_LEFT);
        off += !lamp(&b, LAMP_LEFT);
        CHECK(!lamp(&b, LAMP_RIGHT));
    }
    CHECK(on >= 18 && on <= 22);
    CHECK(off >= 18 && off <= 22);
    b.inputs = 0;
    step(&b, 50);
    CHECK(!lamp(&b, LAMP_LEFT));
}

static void indicator_starts_lit(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.inputs = IN_STALK_RIGHT;
    step(&b, 30);                                /* debounced at the 3rd sample */
    CHECK_EQ(b.c.blink_start_ms, b.t);
    step(&b, 30);                                /* next 50 ms output frame */
    CHECK(lamp(&b, LAMP_RIGHT));
}

/* ---- R2 hazard ---- */
static void hazard_toggles_on_press_not_hold(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.inputs = IN_HAZARD_BTN;
    step(&b, 500);                               /* held: toggles once */
    CHECK(b.a.lamps2 & LAMP2_HAZARD);
    b.inputs = 0;
    step(&b, 100);
    CHECK(b.a.lamps2 & LAMP2_HAZARD);
    b.inputs = IN_HAZARD_BTN;
    step(&b, 100);
    CHECK(!(b.a.lamps2 & LAMP2_HAZARD));
}

static void hazard_blinks_both_in_phase_and_overrides_stalk(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.inputs = IN_HAZARD_BTN;
    step(&b, 100);
    b.inputs = IN_STALK_LEFT;
    for (int i = 0; i < 30; ++i) {
        step(&b, 50);
        CHECK_EQ(lamp(&b, LAMP_LEFT), lamp(&b, LAMP_RIGHT));
    }
}

/* ---- R3 ---- */
static void both_stalk_contacts_is_implausible(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.inputs = IN_STALK_LEFT | IN_STALK_RIGHT;
    for (int i = 0; i < 20; ++i) {
        step(&b, 50);
        CHECK(!lamp(&b, LAMP_LEFT) && !lamp(&b, LAMP_RIGHT));
    }
}

/* ---- R5 beams ---- */
static void beam_logic(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.inputs = IN_MAIN_BEAM;                     /* main selected but lights off */
    step(&b, 100);
    CHECK(!lamp(&b, LAMP_MAIN) && !lamp(&b, LAMP_DIP));
    b.inputs = IN_MAIN_BEAM | IN_DIP_SWITCH;
    step(&b, 100);
    CHECK(lamp(&b, LAMP_MAIN) && lamp(&b, LAMP_DIP));
    b.inputs = IN_FLASH;                         /* flash works with lights off */
    step(&b, 100);
    CHECK(lamp(&b, LAMP_MAIN) && !lamp(&b, LAMP_DIP));
}

/* ---- R10 inputs supervision ---- */
static void input_board_loss_turns_lamps_off(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.inputs = IN_DIP_SWITCH | IN_HAZARD_BTN;
    step(&b, 100);
    CHECK(lamp(&b, LAMP_DIP));
    b.send_inputs = false;
    step(&b, 150);
    CHECK(b.a.lamps2 & LAMP2_INPUT_FAULT);
    CHECK(!lamp(&b, LAMP_DIP) && !lamp(&b, LAMP_LEFT) && !(b.a.lamps2 & LAMP2_HAZARD));
    b.send_inputs = true;
    b.inputs = IN_DIP_SWITCH;
    step(&b, 100);
    CHECK(!(b.a.lamps2 & LAMP2_INPUT_FAULT));
    CHECK(lamp(&b, LAMP_DIP));
}

static void corrupted_input_frames_are_ignored(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.corrupt_inputs = 3;                        /* flips bit 0 = stalk left, on the wire */
    step(&b, 30);
    CHECK_EQ(b.c.stats.inputs_rejected, 3);
    CHECK_EQ(b.c.deb.stable, 0);                 /* the false "left" never reached the debouncer */
    step(&b, 100);
    CHECK(!lamp(&b, LAMP_LEFT) && !(b.a.lamps2 & LAMP2_INPUT_FAULT));
}

static void corrupted_input_stream_is_an_input_fault(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    b.inputs = IN_DIP_SWITCH;
    step(&b, 100);
    b.corrupt_inputs = 1000;                     /* frames still arrive, but none is usable */
    step(&b, 150);
    CHECK(b.a.lamps2 & LAMP2_INPUT_FAULT);
    CHECK(!lamp(&b, LAMP_DIP));
}

/* ---- R6..R9 ECU ---- */
static void ecu_values_shown_after_sync(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 40);
    CHECK_EQ(b.a.speed_dkph, CAN_U16_INVALID);
    CHECK(!lamp(&b, LAMP_ECU_FAULT));            /* start-up grace */
    step(&b, 200);
    CHECK_EQ(b.a.speed_dkph, 500);
    CHECK_EQ(b.a.rpm, 1500);
    CHECK(!lamp(&b, LAMP_ECU_FAULT));
}

static void ecu_never_seen_raises_fault_after_grace(void)
{
    bench_t b;
    bench_init(&b);
    b.send_ecu = false;
    step(&b, 450);
    CHECK(!lamp(&b, LAMP_ECU_FAULT));
    step(&b, 100);
    CHECK(lamp(&b, LAMP_ECU_FAULT));
    CHECK_EQ(b.a.rpm, CAN_U16_INVALID);
}

static void ecu_cut_shows_dashes_not_stale_values(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    CHECK_EQ(b.a.speed_dkph, 500);
    b.send_ecu = false;
    step(&b, 100);
    CHECK_EQ(b.a.speed_dkph, 500);               /* within the 100 ms timeout */
    step(&b, 60);
    CHECK_EQ(b.a.speed_dkph, CAN_U16_INVALID);
    CHECK_EQ(b.a.rpm, CAN_U16_INVALID);
    CHECK(lamp(&b, LAMP_ECU_FAULT));
    CHECK_EQ(b.c.stats.ecu_timeouts, 1);
}

static void ecu_recovers_after_three_good_frames(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    b.send_ecu = false;
    step(&b, 300);
    b.send_ecu = true;
    b.ecu.speed_kph = 60;
    step(&b, 40);                                /* 2 good frames: not yet (R8) */
    CHECK_EQ(b.a.speed_dkph, CAN_U16_INVALID);
    CHECK(lamp(&b, LAMP_ECU_FAULT));
    step(&b, 100);
    CHECK_EQ(b.a.speed_dkph, 600);
    CHECK(!lamp(&b, LAMP_ECU_FAULT));
}

static void corrupted_ecu_frames_are_ignored(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    b.ecu.speed_kph = 200;
    b.corrupt_ecu = 3;
    step(&b, 60);
    CHECK_EQ(b.a.speed_dkph, 500);
    CHECK_EQ(b.c.stats.ecu_rejected_crc, 3);
}

static void corrupted_ecu_stream_times_out(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    b.corrupt_ecu = 1000;
    step(&b, 200);
    CHECK_EQ(b.a.speed_dkph, CAN_U16_INVALID);
    CHECK(lamp(&b, LAMP_ECU_FAULT));
}

static void frozen_alive_counter_times_out(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    b.freeze_ecu_counter = true;                 /* ECU software hung, CAN controller still sending */
    b.ecu.speed_kph = 51;
    step(&b, 200);
    CHECK_EQ(b.a.speed_dkph, CAN_U16_INVALID);
    CHECK(b.c.stats.ecu_rejected_seq >= 5);
}

static void out_of_range_rpm_rejected(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    b.ecu.rpm = 12000;
    step(&b, 200);
    CHECK_EQ(b.a.rpm, CAN_U16_INVALID);
    CHECK(b.c.stats.ecu_rejected_range >= 5);
}

static void implausible_speed_jump_rejected(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    b.ecu.speed_kph = 150;                       /* +100 km/h in 20 ms */
    step(&b, 20);
    CHECK_EQ(b.a.speed_dkph, 500);
    CHECK_EQ(b.c.stats.ecu_rejected_rate, 1);
}

static void check_engine_follows_valid_ecu_only(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    CHECK(!lamp(&b, LAMP_CHECK_ENGINE));
    b.ecu.mil = true;
    step(&b, 60);
    CHECK(lamp(&b, LAMP_CHECK_ENGINE));
    b.ecu.mil = false;
    step(&b, 60);
    CHECK(!lamp(&b, LAMP_CHECK_ENGINE));
    b.ecu.mil = true;
    step(&b, 60);
    b.send_ecu = false;
    step(&b, 200);
    CHECK(!lamp(&b, LAMP_CHECK_ENGINE));
    CHECK(lamp(&b, LAMP_ECU_FAULT));
}

static void single_speed_spike_rejected_but_persistent_change_accepted(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    b.ecu.speed_kph = 150;                       /* +100 km/h in one 20 ms frame */
    step(&b, 20);
    CHECK_EQ(b.a.speed_dkph, 500);               /* the spike never reaches the needle */
    step(&b, 60);                                /* it stays at 150: re-anchored, then accepted */
    CHECK_EQ(b.a.speed_dkph, 1500);
}

static void ecu_start_up_has_no_false_fault(void)
{
    bench_t b;
    bench_init(&b);
    for (int i = 0; i < 6; ++i) {                /* the first 60 ms: frames arrive, not trusted yet */
        step(&b, 10);
        CHECK(!lamp(&b, LAMP_ECU_FAULT));        /* "--" is right, a red fault lamp would be a lie */
    }
    CHECK_EQ(b.a.rpm, CAN_U16_INVALID);
}

static void ecu_needs_three_good_frames_after_a_freeze(void)
{
    /* R8, measured inside the core: after the ECU comes back, values may return only after
     * 3 good frames in a row (60 ms at 20 ms per frame), plus the E2E resync. */
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    b.freeze_ecu_counter = true;                 /* ECU hangs: timeout, gauges "--" */
    step(&b, 200);
    CHECK(!b.c.ecu_valid);
    b.freeze_ecu_counter = false;
    int back_after = -1;
    for (int ms = 10; ms <= 200 && back_after < 0; ms += 10) {
        step(&b, 10);
        if (b.c.ecu_valid) back_after = ms;
    }
    CHECK(back_after >= 60);
}

/* ---- R11, R12 positioning ---- */
static void nav_statuses_map_to_lamps(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 300);
    CHECK_EQ(b.b.text, POS_TXT_OK);
    CHECK_EQ(b.b.heading_cdeg, 9000);
    CHECK(!lamp(&b, LAMP_POS_AMBER) && !lamp(&b, LAMP_POS_RED));
    b.nav.status = NAV_DEGRADED;
    step(&b, 200);
    CHECK_EQ(b.b.text, POS_TXT_DEGRADED);
    CHECK(lamp(&b, LAMP_POS_AMBER));
    CHECK_EQ(b.b.heading_cdeg, 9000);
    b.nav.status = NAV_UNAVAILABLE;
    step(&b, 200);
    CHECK(lamp(&b, LAMP_POS_RED));
    CHECK_EQ(b.b.heading_cdeg, CAN_U16_INVALID);
    b.nav.status = NAV_INIT;
    step(&b, 200);
    CHECK_EQ(b.b.text, POS_TXT_ALIGNING);
    CHECK(lamp(&b, LAMP_POS_AMBER));
    CHECK_EQ(b.b.heading_cdeg, CAN_U16_INVALID);
}

static void nav_loss_is_red_with_no_heading(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 500);
    b.send_nav = false;
    step(&b, 300);
    CHECK_EQ(b.b.text, POS_TXT_OK);               /* 300 ms since last frame: still in time */
    step(&b, 100);
    CHECK_EQ(b.b.text, POS_TXT_NAV_LOST);
    CHECK(lamp(&b, LAMP_POS_RED));
    CHECK_EQ(b.b.heading_cdeg, CAN_U16_INVALID);
}

/* ---- output timing ---- */
static void outputs_every_50_ms(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 1000);
    CHECK_EQ(b.outputs, 20);
}

static void overrun_does_not_burst(void)
{
    cluster_core_t c;
    core_init(&c, 0);
    can_frame_t out[2];
    CHECK_EQ(core_tick(&c, 0, out), 2);
    CHECK_EQ(core_tick(&c, 500, out), 2);        /* stalled 500 ms */
    CHECK_EQ(core_tick(&c, 510, out), 0);        /* no catch-up flood */
    CHECK_EQ(core_tick(&c, 550, out), 2);
}

void suite_core(void)
{
    RUN(debounce_needs_three_stable_samples);
    RUN(debounce_ignores_bounce);
    RUN(debounce_bits_are_independent);
    RUN(blink_90_per_minute_half_duty);
    RUN(left_indicator_blinks);
    RUN(indicator_starts_lit);
    RUN(hazard_toggles_on_press_not_hold);
    RUN(hazard_blinks_both_in_phase_and_overrides_stalk);
    RUN(both_stalk_contacts_is_implausible);
    RUN(beam_logic);
    RUN(input_board_loss_turns_lamps_off);
    RUN(corrupted_input_frames_are_ignored);
    RUN(corrupted_input_stream_is_an_input_fault);
    RUN(ecu_values_shown_after_sync);
    RUN(ecu_never_seen_raises_fault_after_grace);
    RUN(ecu_cut_shows_dashes_not_stale_values);
    RUN(ecu_recovers_after_three_good_frames);
    RUN(corrupted_ecu_frames_are_ignored);
    RUN(corrupted_ecu_stream_times_out);
    RUN(frozen_alive_counter_times_out);
    RUN(out_of_range_rpm_rejected);
    RUN(implausible_speed_jump_rejected);
    RUN(single_speed_spike_rejected_but_persistent_change_accepted);
    RUN(ecu_start_up_has_no_false_fault);
    RUN(check_engine_follows_valid_ecu_only);
    RUN(ecu_needs_three_good_frames_after_a_freeze);
    RUN(nav_statuses_map_to_lamps);
    RUN(nav_loss_is_red_with_no_heading);
    RUN(outputs_every_50_ms);
    RUN(overrun_does_not_burst);
}
