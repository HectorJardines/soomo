#ifndef _ADC_H
#define _ADC_H

#include "main.h"
#include <stdint.h>

#define ADC_CHANNEL_CNT    (16u) 

typedef uint16_t adc_channel_values_t[ADC_CHANNEL_CNT];

void adc_init(void);
void adc_get_channel_values(adc_channel_values_t values);

#endif