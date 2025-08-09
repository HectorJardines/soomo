#include "adc.h"
#include "assert_handler.h"
#include "io.h"
#include "dma.h"
#include "trace.h"
#include "interrupts.h"

#define ADC_CONVERSION_CNT  (4U)
#define ADC_SQR_LENGTH      (ADC_CONVERSION_CNT - 1)
#define NO_OF_CHANNELS_PER_CONV     (ADC_CONVERSION_CNT)
#define DMA_CHANNEL_CNT             (ADC_CONVERSION_CNT)

volatile adc_channel_values_t adc_dma_block;
volatile adc_channel_values_t adc_dma_block_cache;
static const io_e *adc_pins;
static uint8_t adc_pin_count;

static inline void adc_peripheral_control_enable(void)
{
    RCC->APB2ENR |= (RCC_APB2ENR_ADC1EN);
}

static inline void adc_peripheral_control_disable(void)
{
    RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN);
}

static inline void adc_peripheral_enable(void)
{
    ADC1->CR2 |= (ADC_CR2_ADON);
}

static void adc_enable_and_start_conversion(void)
{
    // TRACE("STARTING TRANSFER");
    ADC1->CR2 |= (ADC_CR2_SWSTART);
}

static void adc_set_channel_sampling_time(void)
{
    ADC1->SMPR2 |= (4U << (ADC_CH_4 * 3)) | (4U << (ADC_CH_6 * 3)) | (4U << (ADC_CH_7 * 3)) | (4U << (ADC_CH_8 * 3));
}
static bool initialized = false;
void adc_init(void)
{
    adc_peripheral_control_enable();

    TRACE("IN ADC INIT");
    ASSERT(!initialized);
    adc_pins = io_adc_pins(&adc_pin_count);
    // set ADC peripheral in scan mode and configure adc resolution (CR1 register)
    ADC1->CR1 |= (ADC_RES_10 << ADC_CR1_RES_Pos) | (ENABLE << ADC_CR1_SCAN_Pos);
    // select channels to be scanned and converted (SQRx register)
    ADC1->SQR1 &= ~(ADC_SQR1_L);
    ADC1->SQR1 |= (ADC_SQR_LENGTH << ADC_SQR1_L_Pos);
    // configure channels 4,6,7,8 to be first 4 channels in conversion sequence
    ADC1->SQR3 = (ADC_CH_4 << ADC_SQR3_SQ1_Pos) | (ADC_CH_6 << ADC_SQR3_SQ2_Pos) | (ADC_CH_7 << ADC_SQR3_SQ3_Pos) | (ADC_CH_8 << ADC_SQR3_SQ4_Pos);
    adc_set_channel_sampling_time();
    // set peripheral in continuous mode with EOC bit set at end of sequence conv
    ADC1->CR2 |= (ADC_CR2_CONT) | (ADC_CR2_DDS); // READD DDS
    ADC1->CR2 &= ~(ADC_CR2_EOCS);
    // enable DMA
    ADC1->CR2 |= (ADC_CR2_DMA);
    dma_init((uint32_t)&ADC1->DR, (uint32_t)adc_dma_block);
    // enable end of conversion interrupts
    adc_peripheral_enable();
    adc_enable_and_start_conversion();
    initialized = true;
}

void adc_get_channel_values(adc_channel_values_t values)
{
    TRACE("IN ADC GET CHANNEL VALS");
    __disable_irq();
    for (uint8_t i = 0; i < adc_pin_count; ++i)
        values[i] = adc_dma_block_cache[i];
    __enable_irq();
}

/************************************
 *           ADC IRQ Handling
 ************************************/

void DMA2_Stream0_IRQHandler(void)
{
    // Move conversion from data register to other memory location
    DMA2->LIFCR |= (DMA_LIFCR_CTCIF0);
    for (uint8_t i = 0; i < adc_pin_count; ++i) 
        adc_dma_block_cache[i] = adc_dma_block[i];
    // Restart next conversion
    adc_enable_and_start_conversion();
}
