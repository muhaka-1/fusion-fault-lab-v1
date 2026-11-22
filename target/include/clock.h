#ifndef CLOCK_H
#define CLOCK_H
#include <stdint.h>
int64_t clock_now_us(void);                 /* CLOCK_MONOTONIC */
void clock_sleep_until_us(int64_t t_us);    /* absolute, CLOCK_MONOTONIC */
#endif
