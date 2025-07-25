#include "pwm.h"
#include "defines.h"

struct pwm_channel_confg
{
    bool enabled;
    volatile uint32_t *const ccmr;
    volatile uint32_t *const ccr;
};

static struct pwm_channel_confg pwm_cfgs[] = {
    [PWM_TB6612FNG_LEFT] = {.enabled = false, .ccmr = &TIM2->CCMR1, .ccr = &TIM2->CCR1},
    [PWM_TB6612FNG_RIGHT] = {.enabled = false, .ccmr = &TIM2->CCMR1, .ccr = &TIM2->CCR2}
};

#define PWM_TIMER_FREQ_HZ       (SYS_CLK / TIMER_INPUT_DIVIDER3)
#define PWM_PERIOD_FREQ_HZ      (20000U)
#define PWM_TIMER_TICK_COUNTS   (PWM_TIMER_FREQ_HZ / PWM_PERIOD_FREQ_HZ) // 100 Ticks
#define PWM_TIMER_PERIOD        (PWM_TIMER_TICK_COUNTS - 1)

#define PWM_TIMER_PSC           ((SYS_CLK / (TIMER_INPUT_DIVIDER3 * TIMER_CLK_1MHZ)) - 1)
void tim2_clock_enable(void)
{
    RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN);
}

static bool initialized = false;

void pwm_init(void)
{
    if (initialized)
        return;

    tim2_clock_enable();
    // 1. Configure output pin
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM2->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM2->CCMR1 |= ((0x7U << TIM_CCMR1_OC1M_Pos) | (0x7U << TIM_CCMR1_OC2M_Pos));

    // 2. Set timer period in Auto Reload Reg (count from 0 - 99)
    TIM2->PSC = (PWM_TIMER_PSC);
    TIM2->ARR = (PWM_TIMER_PERIOD);
    initialized = true;
}

static uint8_t pwm_scale_duty_cycle(uint8_t duty_cycle)
{
    // Must scale down duty cycle by .75 because battery rating is 8V, but motors are rated for 6V
    uint8_t scaled_duty_cycle = duty_cycle * 3 / 4;
    return scaled_duty_cycle > 0 ? scaled_duty_cycle : 1;
}

static bool pwm_all_channels_disabled(void)
{
    for (uint8_t channel = 0; channel < NO_OF_PWM_CFGS; channel++)
        if (pwm_cfgs[channel].enabled == true)
            return false;
    
    return true;
}

static bool pwm_enabled = false;
static void pwm_enable(bool enable)
{
    if (pwm_enabled != enable && enable == true)
    { 
        TIM2->CCER |= (TIM_CCER_CC1E);
        TIM2->CR1 |= (TIM_CR1_CEN);
    }
    else if (pwm_enabled != enable && enable == false)
    {
        TIM2->CR1 &= ~(TIM_CR1_CEN);
    }
    pwm_enabled = enable;
}

static void pwm_channel_enable(pwm_e pwm, uint8_t EnOrDi)
{
    if (pwm_cfgs[pwm].enabled != EnOrDi)
    {
        *pwm_cfgs[pwm].ccmr |= (TIM_CCMR1_OC1PE);
        TIM2->CR1 |= (TIM_CR1_ARPE);
        TIM2->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR);

        pwm_cfgs[pwm].enabled = EnOrDi;

        if (EnOrDi == true)
            pwm_enable(true);
        else if (pwm_all_channels_disabled())
            pwm_enable(false);
    }
}

void pwm_set_duty_cycle(pwm_e pwm, uint8_t duty_cycle_percentage)
{
    // Load TIM2 CCR1 register with the duty cycle value
    if (duty_cycle_percentage > 100)
        return;
    
    const bool enable = duty_cycle_percentage > 0;
    if (enable)
    {
        *pwm_cfgs[pwm].ccr = pwm_scale_duty_cycle(duty_cycle_percentage);
    }

    pwm_channel_enable(pwm, enable);
}
