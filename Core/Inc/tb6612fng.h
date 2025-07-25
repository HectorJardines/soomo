#ifndef _TB6612FNG_H
#define _TB6612FNG_H

#include "main.h"

typedef enum
{
    TB6612FNG_LEFT,
    TB6612FNG_RIGHT
} tb6612fng_e;

typedef enum
{
    TB6612FNG_STOP,
    TB6612FNG_FORWARD,   // CLOCKWISE
    TB6612FNG_REVERSE   // COUNTER-CC
} tb6612fng_mode_e;

void tb6612fng_init(void);
void tb6612fng_set_mode(tb6612fng_e tb, tb6612fng_mode_e tb_mode);
void tb6612fng_set_pwm(tb6612fng_e tb, uint8_t duty_cycle);

#endif