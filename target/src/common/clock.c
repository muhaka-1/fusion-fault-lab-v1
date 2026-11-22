#include "clock.h"
#include <errno.h>
#include <time.h>

int64_t clock_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

void clock_sleep_until_us(int64_t t_us)
{
    struct timespec ts = { .tv_sec = (time_t)(t_us / 1000000), .tv_nsec = (long)(t_us % 1000000) * 1000 };
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) { }
}
