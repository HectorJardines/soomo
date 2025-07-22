#ifndef _PWM_H
#define _PWM_H
#include "main.h"

#define NO_OF_PWM_CFGS      2U

/*******************************
 *          PWM ENUMS
 *******************************/

typedef enum
{
    PWM_TB6612FNG_LEFT,
    PWM_TB6612FNG_RIGHT
}pwm_e;

/*******************************
 *          PWM APIs
 *******************************/

void pwm_init(void);
void pwm_set_duty_cycle(pwm_e pwm, uint8_t duty_cycle_percentage);
void tim2_clock_enable(void);

#endif