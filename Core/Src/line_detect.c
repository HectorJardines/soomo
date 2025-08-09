#include "line_detect.h"
#include "assert_handler.h"

#define LINE_DETECT_VOLTAGE_THRESHOLD   (700u) // CHANGE IF THIS DIFFERS ON MY BOARD

static bool initialized = false;
void line_detect_init(void)
{
    ASSERT(!initialized);
    qre113_init();
    initialized = true;
}

static line_pos_e voltage_values_to_position(const bool front_left, const bool front_right, const bool back_left, const bool back_right)
{
    if (front_left)
    {
        if (front_right)
            return LINE_FRONT;
        else if (back_left)
            return LINE_LEFT;
        else if (back_right)
            return LINE_DIAGONAL_LEFT;
        else
            return LINE_FRONT_LEFT;
    }
    else if (front_right)
    {
        if (back_right)
            return LINE_RIGHT;
        else if (back_left)
            return LINE_DIAGONAL_RIGHT;
        else 
            return LINE_FRONT_RIGHT;
    }
    else if (back_left)
    {
        if (back_right)
            return LINE_BACK;
        else
            return LINE_BACK_LEFT;
    }
    else if (back_right)
    {
        return LINE_BACK_RIGHT;
    }
    return LINE_NONE;
}

line_pos_e line_get_positon(void)
{
    struct qre113_voltages line_detect_voltages;
    qre113_read_values(&line_detect_voltages);

    const bool front_left = line_detect_voltages.front_left < LINE_DETECT_VOLTAGE_THRESHOLD;
    const bool front_right = line_detect_voltages.front_right < LINE_DETECT_VOLTAGE_THRESHOLD;
    const bool back_left = line_detect_voltages.back_left < LINE_DETECT_VOLTAGE_THRESHOLD;
    const bool back_right = line_detect_voltages.back_right < LINE_DETECT_VOLTAGE_THRESHOLD;

    return voltage_values_to_position(front_left, front_right, back_left, back_right);
}
