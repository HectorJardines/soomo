#ifndef _DRIVE_H
#define _DRIVE_H

typedef enum
{
    DRIVE_SPEED_SLOW,
    DRIVE_SPEED_MEDIUM,
    DRIVE_SPEED_FAST,
    DRIVE_SPEED_MAX
} drive_speed_e;

typedef enum
{
    DRIVE_DIR_FORWARD,
    DRIVE_DIR_REVERSE,
    DRIVE_DIR_TURN_RIGHT,
    DRIVE_DIR_TURN_LEFT,
    DRIVE_DIR_ARCTURN_SHARP_RIGHT,
    DRIVE_DIR_ARCTURN_SHARP_LEFT,
    DRIVE_DIR_ARCTURN_MID_RIGHT,
    DRIVE_DIR_ARCTURN_MID_LEFT,
    DRIVE_DIR_ARCTURN_WIDE_RIGHT,
    DRIVE_DIR_ARCTURN_WIDE_LEFT
} drive_dir_e;

void drive_init(void);
void drive_stop(void);
void drive_set(drive_speed_e speed, drive_dir_e dir);

#endif