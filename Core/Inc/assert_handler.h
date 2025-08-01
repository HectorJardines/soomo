#ifndef _ASSERT_H
#define _ASSERT_H
#include <stdint.h>

#define ASSERT(expression) \
    do { \
        volatile uint16_t pc; \
        asm volatile("mov %0, pc\n" : "=r" (pc)); \
        if (!(expression)) { \
            assert_handler(pc); \
        } \
    } while (0)

#define ASSERT_INTERRUPT(expression) \
    do { \
        if (!(expression)) { \
            while(1) {} \
        } \
    } while(0)

void assert_handler(uint16_t pc);

#endif