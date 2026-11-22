/* Wire format of the simulated bus (portable C, no sockets): 16 bytes per frame,
 *   id (u32 little endian) | dlc (u8) | 3 x 0 | data[8]
 * Kept apart from canbus_udp.c so it builds and is tested everywhere, Windows included. */
#include "canbus.h"
#include <string.h>

void canbus_pack(const can_frame_t *f, unsigned char out[CANBUS_WIRE_LEN])
{
    memset(out, 0, CANBUS_WIRE_LEN);
    out[0] = (unsigned char)f->id; out[1] = (unsigned char)(f->id >> 8);
    out[2] = (unsigned char)(f->id >> 16); out[3] = (unsigned char)(f->id >> 24);
    out[4] = f->dlc;
    memcpy(&out[8], f->data, 8);
}

int canbus_unpack(const unsigned char *in, long len, can_frame_t *f)
{
    if (len != CANBUS_WIRE_LEN || in[4] > 8) return -1;
    f->id = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
    if (f->id > 0x7FFu) return -1;           /* 11-bit identifiers only */
    f->dlc = in[4];
    memcpy(f->data, &in[8], 8);
    return 0;
}
