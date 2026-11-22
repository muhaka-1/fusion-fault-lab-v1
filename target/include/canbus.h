/* Simulated CAN bus: classic CAN frames carried in UDP multicast datagrams.
 * Every node sees every frame, like a real bus. Replace this file's .c with a
 * SocketCAN / QNX dev-can backend when a real adapter is available.
 *
 * Environment: CANBUS_GROUP (239.255.42.42), CANBUS_PORT (42424),
 *              CANBUS_IF  (interface address to join on, default 127.0.0.1)
 * Wire format: 16 bytes = id (u32 LE) | dlc (u8) | 3 x 0 | data[8]   (canbus_wire.c, portable)
 * Transport:   UDP multicast, TTL 1                                  (canbus_udp.c, POSIX only)
 */
#ifndef CANBUS_H
#define CANBUS_H
#include "can_msg.h"

#define CANBUS_WIRE_LEN 16

typedef struct { int fd; unsigned char addr[16]; } canbus_t;

int canbus_open(canbus_t *bus);                 /* 0 on success, -1 + perror */
int canbus_send(canbus_t *bus, const can_frame_t *f);
/* wait up to timeout_ms (-1 = forever, 0 = poll); 1 = frame, 0 = timeout, -1 = error */
int canbus_recv(canbus_t *bus, can_frame_t *f, int timeout_ms);
void canbus_close(canbus_t *bus);
void canbus_pack(const can_frame_t *f, unsigned char out[CANBUS_WIRE_LEN]);
int canbus_unpack(const unsigned char *in, long len, can_frame_t *f);

#endif
