/* End-to-end protection for 8-byte CAN payloads.
 *
 * Modelled on AUTOSAR E2E Profile 1 (simplified, not certified):
 *   byte 0       CRC-8 SAE J1850 over the 16-bit data ID and bytes 1..7
 *   byte 1 0..3  alive counter, 0..14 (15 is never sent)
 *   byte 1 4..7  free for message flags
 * The receiver classifies every frame; timeouts are handled by the caller,
 * which knows the expected period.
 */
#ifndef E2E_H
#define E2E_H
#include <stdbool.h>
#include <stdint.h>

#define E2E_COUNTER_MOD 15u

typedef enum {
    E2E_OK = 0,          /* next counter value */
    E2E_OK_SOME_LOST,    /* counter jumped by 2..max_delta: frames lost, data still usable */
    E2E_INITIAL,         /* first frame, or first frame after a resync: not yet trusted */
    E2E_REPEATED,        /* same counter again: stale sender, discard */
    E2E_WRONG_SEQUENCE,  /* counter jumped too far: discard and resync */
    E2E_BAD_CRC,         /* corrupted: discard */
    E2E_BAD_COUNTER      /* counter value 15: invalid */
} e2e_status_t;

typedef struct {
    uint8_t counter;
} e2e_tx_t;

typedef struct {
    uint8_t max_delta;   /* largest accepted counter jump (1 = no loss tolerated) */
    uint8_t sync_needed; /* consecutive in-sequence frames needed after (re)sync */
    bool has_last;
    uint8_t last;
    uint8_t sync_left;
    uint8_t last_delta;  /* counter jump of the last in-sequence frame (1 = none lost) */
} e2e_rx_t;

uint8_t e2e_crc8(const uint8_t *data, unsigned len, uint8_t crc);
uint8_t e2e_crc8_frame(uint16_t data_id, const uint8_t payload[8]);
void e2e_protect(e2e_tx_t *tx, uint16_t data_id, uint8_t payload[8]);
void e2e_rx_init(e2e_rx_t *rx, uint8_t max_delta, uint8_t sync_needed);
e2e_status_t e2e_check(e2e_rx_t *rx, uint16_t data_id, const uint8_t payload[8]);
bool e2e_usable(e2e_status_t s);
const char *e2e_status_name(e2e_status_t s);

#endif
