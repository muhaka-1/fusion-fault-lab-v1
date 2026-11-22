#include "e2e.h"

/* CRC-8 SAE J1850: poly 0x1D, init 0xFF, final xor 0xFF. Check("123456789") = 0x4B. */
uint8_t e2e_crc8(const uint8_t *data, unsigned len, uint8_t crc)
{
    for (unsigned i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (uint8_t)((crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x1Du) : (uint8_t)(crc << 1));
    }
    return crc;
}

uint8_t e2e_crc8_frame(uint16_t data_id, const uint8_t payload[8])
{
    const uint8_t id[2] = { (uint8_t)(data_id & 0xFFu), (uint8_t)(data_id >> 8) };
    uint8_t crc = e2e_crc8(id, 2, 0xFFu);
    crc = e2e_crc8(&payload[1], 7, crc);
    return (uint8_t)(crc ^ 0xFFu);
}

void e2e_protect(e2e_tx_t *tx, uint16_t data_id, uint8_t payload[8])
{
    payload[1] = (uint8_t)((payload[1] & 0xF0u) | (tx->counter & 0x0Fu));
    payload[0] = e2e_crc8_frame(data_id, payload);
    tx->counter = (uint8_t)((tx->counter + 1u) % E2E_COUNTER_MOD);
}

void e2e_rx_init(e2e_rx_t *rx, uint8_t max_delta, uint8_t sync_needed)
{
    rx->max_delta = max_delta ? max_delta : 1u;
    rx->sync_needed = sync_needed;
    rx->has_last = false;
    rx->last = 0;
    rx->sync_left = sync_needed;
    rx->last_delta = 1;
}

e2e_status_t e2e_check(e2e_rx_t *rx, uint16_t data_id, const uint8_t payload[8])
{
    if (payload[0] != e2e_crc8_frame(data_id, payload))
        return E2E_BAD_CRC;
    const uint8_t c = payload[1] & 0x0Fu;
    if (c >= E2E_COUNTER_MOD)
        return E2E_BAD_COUNTER;
    if (!rx->has_last) {
        rx->has_last = true;
        rx->last = c;
        rx->sync_left = rx->sync_needed;
        rx->last_delta = 1;
        return E2E_INITIAL;
    }
    const uint8_t delta = (uint8_t)((c + E2E_COUNTER_MOD - rx->last) % E2E_COUNTER_MOD);
    if (delta == 0)
        return E2E_REPEATED;
    rx->last = c;
    rx->last_delta = delta;
    if (delta > rx->max_delta) {
        rx->sync_left = rx->sync_needed;
        return E2E_WRONG_SEQUENCE;
    }
    if (rx->sync_left > 0) {
        rx->sync_left--;
        return E2E_INITIAL;
    }
    return delta == 1 ? E2E_OK : E2E_OK_SOME_LOST;
}

bool e2e_usable(e2e_status_t s)
{
    return s == E2E_OK || s == E2E_OK_SOME_LOST;
}

const char *e2e_status_name(e2e_status_t s)
{
    switch (s) {
    case E2E_OK: return "OK";
    case E2E_OK_SOME_LOST: return "OK_SOME_LOST";
    case E2E_INITIAL: return "INITIAL";
    case E2E_REPEATED: return "REPEATED";
    case E2E_WRONG_SEQUENCE: return "WRONG_SEQUENCE";
    case E2E_BAD_CRC: return "BAD_CRC";
    case E2E_BAD_COUNTER: return "BAD_COUNTER";
    }
    return "?";
}
