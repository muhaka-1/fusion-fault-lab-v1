/* e2e_demo: watch the E2E protection work, frame by frame.
 *
 *   e2e_demo              a normal stream, then each kind of fault
 *   e2e_demo 0x200 5      protect 5 frames of message 0x200 and print their bytes
 *
 * Use it to test by hand what the unit tests check automatically.
 */
#include "e2e.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void show(const char *what, uint16_t id, const uint8_t p[8], e2e_status_t s)
{
    printf("%-34s id 0x%03X  [", what, id);
    for (int i = 0; i < 8; ++i) printf(i ? " %02X" : "%02X", p[i]);
    printf("]  crc %02X ctr %2u  ->  %s\n", p[0], p[1] & 0x0Fu, e2e_status_name(s));
}

static void payload(uint8_t p[8], int k)
{
    memset(p, 0, 8);
    p[2] = (uint8_t)(0x10 + k);       /* pretend signal data that changes every frame */
    p[3] = 0xAB;
}

int main(int argc, char **argv)
{
    if (argc == 3) {
        const uint16_t id = (uint16_t)strtoul(argv[1], NULL, 0);
        const int n = atoi(argv[2]);
        e2e_tx_t tx = { 0 };
        e2e_rx_t rx;
        e2e_rx_init(&rx, 2, 0);
        for (int k = 0; k < n; ++k) {
            uint8_t p[8];
            payload(p, k);
            e2e_protect(&tx, id, p);
            show("frame", id, p, e2e_check(&rx, id, p));
        }
        return 0;
    }

    const uint16_t id = 0x200;
    e2e_tx_t tx = { 0 };
    e2e_rx_t rx;
    uint8_t p[8];
    e2e_rx_init(&rx, 2, 1);           /* tolerate 1 lost frame (jump of 2), need 1 good frame after a resync */
    printf("receiver: max counter jump 2, 1 frame to resync\n\n");

    for (int k = 0; k < 4; ++k) { payload(p, k); e2e_protect(&tx, id, p); show("normal frame", id, p, e2e_check(&rx, id, p)); }

    payload(p, 4); e2e_protect(&tx, id, p); p[3] ^= 0x01;
    show("one data bit flipped on the wire", id, p, e2e_check(&rx, id, p));

    payload(p, 5); e2e_protect(&tx, id, p);
    show("next frame (counter skipped 1)", id, p, e2e_check(&rx, id, p));

    show("same frame received again", id, p, e2e_check(&rx, id, p));

    payload(p, 6); e2e_protect(&tx, id, p);
    show("right bytes, wrong message id", 0x210, p, e2e_check(&rx, 0x210, p));

    for (int k = 0; k < 4; ++k) { payload(p, 7 + k); e2e_protect(&tx, id, p); }   /* 4 frames lost */
    payload(p, 11); e2e_protect(&tx, id, p);
    show("after 4 lost frames", id, p, e2e_check(&rx, id, p));
    payload(p, 12); e2e_protect(&tx, id, p);
    show("resync frame", id, p, e2e_check(&rx, id, p));
    payload(p, 13); e2e_protect(&tx, id, p);
    show("back in sequence", id, p, e2e_check(&rx, id, p));

    printf("\nusable for the cluster: OK and OK_SOME_LOST only\n");
    return 0;
}
