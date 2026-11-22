/* Cluster core: the only place that decides what the instrument cluster shows.
 *
 * Inputs (E2E-protected CAN frames):
 *   INPUTS    raw stalk/switch levels from the input board, 100 Hz        (M9)
 *   ENGINE    rpm, vehicle speed, MIL request from the ECU, 50 Hz        (M10)
 *   NAV_STATE positioning status and heading from the nav process, 10 Hz (M10)
 * Output: CLUSTER_A (gauges + lamps) and CLUSTER_B (positioning) every 50 ms.
 *
 * Rules (docs/cluster_rules.md). M9 implements:
 *   R1  every input is debounced: a level must be stable for 3 samples (30 ms)
 *   R2  hazard switch toggles on its debounced rising edge; hazard overrides the stalk
 *   R3  stalk left+right at once is implausible -> no indicator
 *   R4  indicators blink at 90 /min, 50 % duty, starting lit; restart phase on change
 *   R5  main-beam lamp: dip on and main selected, or flash held
 *   R10 input board silent > 100 ms -> INPUT fault, input-driven lamps off
 *   R13 a value is never shown stale: anything without fresh valid data is "--"
 *       (until M10 there is no ECU or nav data at all, so gauges and heading are "--")
 *
 * The core never reads a clock. Every function gets the time as an argument, so tests can
 * run hours of driving in microseconds, and the same code runs on Linux and QNX.
 */
#ifndef CLUSTER_CORE_H
#define CLUSTER_CORE_H
#include <stdbool.h>
#include <stdint.h>
#include "can_msg.h"
#include "e2e.h"

#define CORE_TICK_MS 10
#define CORE_OUT_PERIOD_MS 50
#define DEBOUNCE_SAMPLES 3
#define BLINK_PERIOD_MS 667          /* 60000 / 90 */
#define BLINK_ON_MS 333
#define INPUT_TIMEOUT_MS 100

typedef struct {
    uint8_t stable;                  /* the debounced levels, one bit per input */
    uint8_t count[6];                /* how many samples in a row each bit has disagreed */
} debounce_t;

typedef enum { IND_NONE = 0, IND_LEFT, IND_RIGHT } indicator_t;

typedef struct {
    uint32_t inputs_rejected;        /* INPUTS frames that failed the E2E check */
    uint32_t input_timeouts;
} core_stats_t;

typedef struct {
    int64_t now_ms, start_ms;
    /* inputs */
    debounce_t deb;
    e2e_rx_t rx_inputs;
    int64_t inputs_last_ms;
    bool inputs_seen, input_fault;
    bool hazard;
    indicator_t ind;
    int64_t blink_start_ms;
    bool blink_active;
    /* output */
    e2e_tx_t tx_a, tx_b;
    int64_t last_out_ms;
    bool out_started;
    core_stats_t stats;
} cluster_core_t;

void core_init(cluster_core_t *c, int64_t now_ms);
void core_on_frame(cluster_core_t *c, const can_frame_t *f, int64_t now_ms);
/* advance time; returns number of output frames (0 or 2) written to out[] */
int core_tick(cluster_core_t *c, int64_t now_ms, can_frame_t out[2]);
/* what would be sent now (for tests and logging) */
void core_outputs(const cluster_core_t *c, msg_cluster_a_t *a, msg_cluster_b_t *b);

/* building blocks, exposed for unit tests */
void debounce_init(debounce_t *d);
uint8_t debounce_step(debounce_t *d, uint8_t raw);
bool blink_lit(int64_t start_ms, int64_t now_ms);

#endif
