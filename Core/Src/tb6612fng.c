#include "tb6612fng.h"
#include "io.h"
#include "pwm.h"

struct ctrl_pins
{
    io_e cc1;
    io_e cc2;
};

static struct ctrl_pins cc_pin_cfgs[] = {
    [TB6612FNG_LEFT] = {IO_MOTORS_LEFT_CC1, IO_MOTORS_LEFT_CC2},
    [TB6612FNG_RIGHT] = {IO_MOTORS_RIGHT_CC1, IO_MOTORS_RIGHT_CC2}
};

static bool initialized = false;
void tb6612fng_init(void)
{
    if (initialized)
        return;
    pwm_init();
    initialized = true;
}

void tb6612fng_set_mode(tb6612fng_e tb, tb6612fng_mode_e tb_mode)
{
    switch(tb_mode)
    {
        case TB6612FNG_STOP:
            // io_write_pin(cc_pin_cfgs[tb].cc1, LOW);
            io_write_pin(cc_pin_cfgs[tb].cc2, LOW);
            break;
        case TB6612FNG_FORWARD:
            io_write_pin(cc_pin_cfgs[tb].cc1, HIGH);
            io_write_pin(cc_pin_cfgs[tb].cc2, LOW);
            break;
        case TB6612FNG_REVERSE:
            io_write_pin(cc_pin_cfgs[tb].cc1, LOW);
            io_write_pin(cc_pin_cfgs[tb].cc2, HIGH);
            break;
    }
}

void tb6612fng_set_pwm(tb6612fng_e tb, uint8_t duty_cycle)
{
    pwm_set_duty_cycle((pwm_e)tb, duty_cycle);
}
