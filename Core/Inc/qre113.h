#ifndef _QRE113_H
#define _QRE113_H

#include <stdint.h>
#include "main.h"

typedef enum
{
    LINE_DETECT_FRONT_LEFT_VAL_IDX,
    LINE_DETECT_FRONT_RIGHT_VAL_IDX,
    LINE_DETECT_BACK_LEFT_VAL_IDX = 3,
    LINE_DETECT_BACK_RIGHT_VAL_IDX = 2
} io_adc_value_idx_e;

struct qre113_voltages
{
    /* data */
    uint16_t front_left;
    uint16_t front_right;
    uint16_t back_left;
    uint16_t back_right;
};

void qre113_init(void);
void qre113_read_values(struct qre113_voltages *voltages);

#endif