#include "test.h"
#include "e2e.h"
#include <string.h>

static void frame(uint8_t p[8], uint8_t fill)
{
    memset(p, fill, 8);
    p[1] = 0;
}

static void crc_check_value(void)
{
    const uint8_t s[] = "123456789";
    CHECK_EQ(e2e_crc8(s, 9, 0xFF) ^ 0xFF, 0x4B);   /* CRC-8/SAE-J1850 catalogue check value */
}

static void protect_then_check_ok(void)
{
    e2e_tx_t tx = { 0 };
    e2e_rx_t rx;
    e2e_rx_init(&rx, 1, 0);
    uint8_t p[8];
    for (int i = 0; i < 40; ++i) {
        frame(p, (uint8_t)i);
        e2e_protect(&tx, 0x200, p);
        const e2e_status_t s = e2e_check(&rx, 0x200, p);
        CHECK_EQ(s, i == 0 ? E2E_INITIAL : E2E_OK);
    }
}

static void counter_wraps_0_to_14(void)
{
    e2e_tx_t tx = { 0 };
    uint8_t p[8];
    for (int i = 0; i < 31; ++i) {
        frame(p, 0);
        e2e_protect(&tx, 1, p);
        CHECK_EQ(p[1] & 0x0F, i % 15);
    }
}

static void flags_nibble_is_kept_and_protected(void)
{
    e2e_tx_t tx = { 7 };
    e2e_rx_t rx;
    e2e_rx_init(&rx, 1, 0);
    uint8_t p[8];
    frame(p, 0);
    p[1] = 0x30;
    e2e_protect(&tx, 0x120, p);
    CHECK_EQ(p[1], 0x37);
    p[1] ^= 0x10;                       /* flip a flag bit */
    CHECK_EQ(e2e_check(&rx, 0x120, p), E2E_BAD_CRC);
}

static void every_single_bit_flip_is_caught(void)
{
    e2e_tx_t tx = { 0 };
    uint8_t p[8];
    frame(p, 0x5A);
    e2e_protect(&tx, 0x200, p);
    int caught = 0;
    for (int byte = 0; byte < 8; ++byte)
        for (int bit = 0; bit < 8; ++bit) {
            uint8_t q[8];
            memcpy(q, p, 8);
            q[byte] ^= (uint8_t)(1u << bit);
            e2e_rx_t rx;
            e2e_rx_init(&rx, 1, 0);
            caught += e2e_check(&rx, 0x200, q) == E2E_BAD_CRC;
        }
    CHECK_EQ(caught, 64);
}

static void data_id_is_part_of_crc(void)
{
    e2e_tx_t tx = { 0 };
    e2e_rx_t rx;
    e2e_rx_init(&rx, 1, 0);
    uint8_t p[8];
    frame(p, 1);
    e2e_protect(&tx, 0x200, p);
    CHECK_EQ(e2e_check(&rx, 0x210, p), E2E_BAD_CRC);  /* same bytes, wrong message: rejected */
}

static void repeated_frame_detected(void)
{
    e2e_tx_t tx = { 0 };
    e2e_rx_t rx;
    e2e_rx_init(&rx, 1, 0);
    uint8_t p[8];
    frame(p, 2);
    e2e_protect(&tx, 5, p);
    CHECK_EQ(e2e_check(&rx, 5, p), E2E_INITIAL);
    CHECK_EQ(e2e_check(&rx, 5, p), E2E_REPEATED);
    CHECK_EQ(e2e_check(&rx, 5, p), E2E_REPEATED);
}

static void lost_frames_within_tolerance(void)
{
    e2e_tx_t tx = { 0 };
    e2e_rx_t rx;
    e2e_rx_init(&rx, 3, 0);
    uint8_t p[8];
    frame(p, 0); e2e_protect(&tx, 9, p); e2e_check(&rx, 9, p);
    frame(p, 0); e2e_protect(&tx, 9, p);                 /* lost */
    frame(p, 0); e2e_protect(&tx, 9, p);
    CHECK_EQ(e2e_check(&rx, 9, p), E2E_OK_SOME_LOST);
    CHECK_EQ(rx.last_delta, 2);
    CHECK(e2e_usable(E2E_OK_SOME_LOST));
}

static void wrong_sequence_then_resync(void)
{
    e2e_tx_t tx = { 0 };
    e2e_rx_t rx;
    e2e_rx_init(&rx, 2, 2);
    uint8_t p[8];
    for (int i = 0; i < 3; ++i) { frame(p, 0); e2e_protect(&tx, 9, p); e2e_check(&rx, 9, p); }
    frame(p, 0); e2e_protect(&tx, 9, p); CHECK_EQ(e2e_check(&rx, 9, p), E2E_OK);
    for (int i = 0; i < 5; ++i) { frame(p, 0); e2e_protect(&tx, 9, p); }   /* 5 lost > max 2 */
    frame(p, 0); e2e_protect(&tx, 9, p);
    CHECK_EQ(e2e_check(&rx, 9, p), E2E_WRONG_SEQUENCE);
    frame(p, 0); e2e_protect(&tx, 9, p); CHECK_EQ(e2e_check(&rx, 9, p), E2E_INITIAL);
    frame(p, 0); e2e_protect(&tx, 9, p); CHECK_EQ(e2e_check(&rx, 9, p), E2E_INITIAL);
    frame(p, 0); e2e_protect(&tx, 9, p); CHECK_EQ(e2e_check(&rx, 9, p), E2E_OK);
}

static void counter_15_is_invalid(void)
{
    e2e_rx_t rx;
    e2e_rx_init(&rx, 1, 0);
    uint8_t p[8];
    frame(p, 0);
    p[1] = 0x0F;
    p[0] = e2e_crc8_frame(3, p);
    CHECK_EQ(e2e_check(&rx, 3, p), E2E_BAD_COUNTER);
}

void suite_e2e(void)
{
    RUN(crc_check_value);
    RUN(protect_then_check_ok);
    RUN(counter_wraps_0_to_14);
    RUN(flags_nibble_is_kept_and_protected);
    RUN(every_single_bit_flip_is_caught);
    RUN(data_id_is_part_of_crc);
    RUN(repeated_frame_detected);
    RUN(lost_frames_within_tolerance);
    RUN(wrong_sequence_then_resync);
    RUN(counter_15_is_invalid);
}
