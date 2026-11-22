/* UDP multicast transport (POSIX sockets: Linux, WSL, QNX). The wire format is in canbus_wire.c.
 * struct ip_mreq is a BSD extension, outside strict POSIX: enable it on glibc and QNX */
#define _DEFAULT_SOURCE
#define _QNX_SOURCE
#include "canbus.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static const char *env_or(const char *k, const char *d)
{
    const char *v = getenv(k);
    return (v && *v) ? v : d;
}

int canbus_open(canbus_t *bus)
{
    const char *group = env_or("CANBUS_GROUP", "239.255.42.42");
    const char *ifa = env_or("CANBUS_IF", "127.0.0.1");
    const int port = atoi(env_or("CANBUS_PORT", "42424"));
    struct in_addr g, i;
    if (inet_pton(AF_INET, group, &g) != 1 || inet_pton(AF_INET, ifa, &i) != 1 || port <= 0 || port > 65535) {
        fprintf(stderr, "canbus: bad CANBUS_GROUP/CANBUS_IF/CANBUS_PORT\n");
        return -1;
    }
    bus->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (bus->fd < 0) { perror("canbus: socket"); return -1; }
    const int one = 1;
    setsockopt(bus->fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
#ifdef SO_REUSEPORT
    setsockopt(bus->fd, SOL_SOCKET, SO_REUSEPORT, &one, sizeof one);
#endif
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(bus->fd, (struct sockaddr *)&a, sizeof a) < 0) { perror("canbus: bind"); close(bus->fd); return -1; }
    struct ip_mreq mr;
    mr.imr_multiaddr = g;
    mr.imr_interface = i;
    if (setsockopt(bus->fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mr, sizeof mr) < 0) { perror("canbus: join group"); close(bus->fd); return -1; }
    const unsigned char loop = 1, ttl = 1;
    setsockopt(bus->fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof loop);
    setsockopt(bus->fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof ttl);
    setsockopt(bus->fd, IPPROTO_IP, IP_MULTICAST_IF, &i, sizeof i);
    a.sin_addr = g;
    memcpy(bus->addr, &a, sizeof a);
    return 0;
}

int canbus_send(canbus_t *bus, const can_frame_t *f)
{
    unsigned char w[CANBUS_WIRE_LEN];
    canbus_pack(f, w);
    const ssize_t n = sendto(bus->fd, w, sizeof w, 0, (const struct sockaddr *)bus->addr, sizeof(struct sockaddr_in));
    return n == (ssize_t)sizeof w ? 0 : -1;
}

int canbus_recv(canbus_t *bus, can_frame_t *f, int timeout_ms)
{
    for (;;) {
        struct pollfd p = { .fd = bus->fd, .events = POLLIN, .revents = 0 };
        const int r = poll(&p, 1, timeout_ms);
        if (r < 0) return errno == EINTR ? 0 : -1;
        if (r == 0) return 0;
        unsigned char w[64];
        const ssize_t n = recv(bus->fd, w, sizeof w, 0);
        if (n < 0) return errno == EINTR ? 0 : -1;
        if (canbus_unpack(w, (long)n, f) == 0) return 1;
        timeout_ms = 0;                       /* malformed datagram: drop, check for more */
    }
}

void canbus_close(canbus_t *bus)
{
    if (bus->fd >= 0) close(bus->fd);
    bus->fd = -1;
}
