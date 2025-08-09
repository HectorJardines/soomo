#ifndef _LINE_DETECT_H
#define _LINE_DETECT_H

#include "qre113.h"
#include <stdint.h>

typedef enum
{
    LINE_NONE,
    LINE_FRONT,
    LINE_BACK,
    LINE_LEFT,
    LINE_RIGHT,
    LINE_FRONT_LEFT,
    LINE_FRONT_RIGHT,
    LINE_BACK_LEFT,
    LINE_BACK_RIGHT,
    LINE_DIAGONAL_LEFT,
    LINE_DIAGONAL_RIGHT
} line_pos_e;

void line_detect_init(void);
line_pos_e line_get_positon(void);

#endif