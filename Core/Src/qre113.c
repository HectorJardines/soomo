#include "qre113.h"
#include <stdbool.h>
#include "assert_handler.h"
#include "adc.h"

static bool initialized;
void qre113_init(void)
{
    ASSERT(!initialized);
    adc_init();
    initialized = true;
}
void qre113_read_values(struct qre113_voltages *voltages)
{
    adc_channel_values_t values;
    adc_get_channel_values(values);

    voltages->front_left = values[LINE_DETECT_FRONT_LEFT_VAL_IDX];
    voltages->front_right = values[LINE_DETECT_FRONT_RIGHT_VAL_IDX];
    voltages->back_left = values[LINE_DETECT_BACK_LEFT_VAL_IDX];
    voltages->back_right = values[LINE_DETECT_BACK_RIGHT_VAL_IDX];
}