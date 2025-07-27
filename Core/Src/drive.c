#include "drive.h"
#include "tb6612fng.h"
#include "defines.h"

struct drive_speeds
{
    int8_t left;
    int8_t right;
};

/*
* Direction modes come in pairs (primary / secondary), to retrieve the primary mode
* we subtract the secondary enum value by the mod_2 of said value. this allows us
* to limit code repetition and avoid typos
*/
#define DRIVE_PRIMARY_DIRECTION(direction)  (direction - MOD_2(direction))

static const struct drive_speeds primary_drive_modes[][4] =
{
    [DRIVE_DIR_FORWARD] = {
        [DRIVE_SPEED_SLOW] = {25, 25},
        [DRIVE_SPEED_MEDIUM] = {40, 40},
        [DRIVE_SPEED_FAST] = {55, 55},
        [DRIVE_SPEED_MAX] = {100, 100}
    },

    [DRIVE_DIR_TURN_RIGHT] = {
        [DRIVE_SPEED_SLOW] = {25, -25},
        [DRIVE_SPEED_MEDIUM] = {50, -50},
        [DRIVE_SPEED_FAST] = {60, -60},
        [DRIVE_SPEED_MAX] = {100, -100}
    },

    [DRIVE_DIR_ARCTURN_SHARP_RIGHT] = {
        [DRIVE_SPEED_SLOW] = {25, -10},
        [DRIVE_SPEED_MEDIUM] = {40, -10},
        [DRIVE_SPEED_FAST] = {75, -25},
        [DRIVE_SPEED_MAX] = {100, -20}
    },

    [DRIVE_DIR_ARCTURN_MID_RIGHT] = {
        [DRIVE_SPEED_SLOW] = {25, 15},
        [DRIVE_SPEED_MEDIUM] = {50, 25},
        [DRIVE_SPEED_FAST] = {70, 35},
        [DRIVE_SPEED_MAX] = {100, 50}
    },

    [DRIVE_DIR_ARCTURN_WIDE_RIGHT] = {
        [DRIVE_SPEED_SLOW] = {25, 20},
        [DRIVE_SPEED_MEDIUM] = {50, 40},
        [DRIVE_SPEED_FAST] = {70, 60},
        [DRIVE_SPEED_MAX] = {100, 80}
    }
};

static void drive_inverse_speeds(int8_t *speed_left, int8_t *speed_right)
{
    if (*speed_left == *speed_right)
    {
        *speed_left = -*speed_left;
        *speed_right = -*speed_right;
    }
    else // swap values 
    {
        int8_t tmp = *speed_left;
        *speed_left = *speed_right;
        *speed_right = tmp;
    }
}

static bool initialized = false;
void drive_init(void)
{
    if (initialized)
        return;
    tb6612fng_init();
}

void drive_stop(void)
{
    tb6612fng_set_mode(TB6612FNG_LEFT, TB6612FNG_STOP);
    tb6612fng_set_mode(TB6612FNG_RIGHT, TB6612FNG_STOP);

    tb6612fng_set_pwm(TB6612FNG_LEFT, 0);
    tb6612fng_set_pwm(TB6612FNG_RIGHT, 0);
}

void drive_set(drive_speed_e speed, drive_dir_e dir)
{
    drive_dir_e primary_drive_dir = DRIVE_PRIMARY_DIRECTION(dir);
    int8_t speed_left = primary_drive_modes[primary_drive_dir][speed].left;
    int8_t speed_right = primary_drive_modes[primary_drive_dir][speed].right;

    if (dir != primary_drive_dir)
        drive_inverse_speeds(&speed_left, &speed_right);

    const tb6612fng_mode_e mode_left = speed_left > 0 ? TB6612FNG_FORWARD : TB6612FNG_REVERSE;
    const tb6612fng_mode_e mode_right = speed_right > 0 ? TB6612FNG_FORWARD : TB6612FNG_REVERSE;

    tb6612fng_set_mode(TB6612FNG_LEFT, mode_left);
    tb6612fng_set_mode(TB6612FNG_RIGHT, mode_right);

    tb6612fng_set_pwm(TB6612FNG_LEFT, ABS(speed_left));
    tb6612fng_set_pwm(TB6612FNG_RIGHT, ABS(speed_right));
}
