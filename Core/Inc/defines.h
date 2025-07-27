#ifndef _DEFINES_H
#define _DEFINES_H

#define TIMER_CLK_1MHZ       1000000UL
#define SYS_CLK              16000000UL
#define TIMER_INPUT_DIVIDER3 8

#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof(arr[0]))
#define MOD_2(n)            (n & 1)
#define ABS(n)              ((n) >= 0 ? n : -(n))

#endif