#ifndef _ADC_H
#define _ADC_H

#include "main.h"
#include <stdint.h>

#define ADC_CHANNEL_CNT    (16u)
#define ADC_IRQ_NO         (18U)
typedef uint16_t adc_channel_values_t[ADC_CHANNEL_CNT];

/*****************************
 *          ADC ENUMS
 *****************************/
typedef enum
{
    ADC_RES_12,
    ADC_RES_10,
    ADC_RES_8,
    ADC_RES_6
} adc_res_e;

typedef enum
{
    ADC_CH_0,
    ADC_CH_1,
    ADC_CH_2,
    ADC_CH_3,
    ADC_CH_4,
    ADC_CH_5,
    ADC_CH_6,
    ADC_CH_7,
    ADC_CH_8,
    ADC_CH_9,
    ADC_CH_10,
    ADC_CH_11,
    ADC_CH_12,
    ADC_CH_13,
    ADC_CH_14,
    ADC_CH_15,
    ADC_CH_16
}   adc_ch_e;

/*************************************
 *              ADC APIs
 *************************************/
void adc_init(void);
void adc_get_channel_values(adc_channel_values_t values);

#endif