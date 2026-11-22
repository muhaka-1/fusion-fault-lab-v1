/* M9: cluster core input rules R1-R5, R10. Every test drives the real core through a fake bench
 * with its own clock: no sleeping, no bus, fully repeatable. */
#include "test.h"
#include "cluster_core.h"
#include <string.h>

/* A fake bench: input board every 10 ms, core ticks every 10 ms. */
typedef struct {
    cluster_core_t c;
    int64_t t;
    e2e_tx_t ti;
    uint8_t inputs;
    bool send_inputs;
    int corrupt_inputs;    /* corrupt the next N INPUTS frames */
    msg_cluster_a_t a;
    msg_cluster_b_t b;
    int outputs;
} bench_t;

static void bench_init(bench_t *b)
{
    memset(b, 0, sizeof *b);
    b->t = 1000;
    core_init(&b->c, b->t);
    b->send_inputs = true;
}

static void step(bench_t *b, int ms)
{
    for (int i = 0; i < ms; i += 10) {
        b->t += 10;
        can_frame_t f, out[2];
        if (b->send_inputs) {
            msg_inputs_t m = { b->inputs };
            msg_encode_inputs(&f, &m);
            e2e_protect(&b->ti, (uint16_t)f.id, f.data);
            if (b->corrupt_inputs > 0) { f.data[2] ^= 0x01; b->corrupt_inputs--; }
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

/* ---- R13 until M10: no ECU and no nav data means "--" ---- */
static void gauges_and_heading_are_dashes_without_data(void)
{
    bench_t b;
    bench_init(&b);
    step(&b, 200);
    CHECK_EQ(b.a.speed_dkph, CAN_U16_INVALID);
    CHECK_EQ(b.a.rpm, CAN_U16_INVALID);
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
    RUN(gauges_and_heading_are_dashes_without_data);
    RUN(outputs_every_50_ms);
    RUN(overrun_does_not_burst);
}
