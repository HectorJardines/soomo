#include "main.h"
#include "uart.h"
#include "io.h"
#include "../external/printf/printf.h"
#include "trace.h"
#include "assert_handler.h"
#include "adc.h"

static void test_setup(void)
{
    SystemClock_Config();
    IO_Init();
}

static void test_io_led(void)
{
    test_setup();
    while (1)
    {
        HAL_Delay(500);
        GPIOA->ODR ^= (0x1U << GPIO_ODR_OD5_Pos);
    }
}

static void test_assert_led(void)
{
    test_setup();
    while(1)
    {
        ASSERT(0);
    }
}

static void test_uart(void)
{
    test_setup();
    usart_init();
    while(1)
    {
        _putchar('H');
        _putchar('e');
        _putchar('l');
        _putchar('l');
        _putchar('o');
        _putchar('\n');
        for (volatile int i = 0; i < 350000; ++i);
    }
}

static void test_trace(void)
{
    test_setup();
    trace_init();
    while(1)
    {
        TRACE("Hello World %d", 2025);
        for (volatile int i = 0; i < 350000; ++i);
    }
}

static void test_adc(void)
{
    test_setup();
    trace_init();
    adc_init();
    while(1)
    {
        adc_channel_values_t vals = {0};
        adc_get_channel_values(vals);
        for (uint8_t i = 0; i < ADC_CHANNEL_CNT; ++i)
        {
            TRACE("ADC VALUE ON CHANNEL %d : %d\n", i, vals[i]);
        }
        // if (ADC1->SR & ADC_SR_EOC) {
        //     uint16_t sample = ADC1->DR;
        //     TRACE("SAMPLE IS: %d", sample);
        // }
        // while(!(DMA2->LISR & DMA_LISR_TCIF0))
        //     TRACE("DMA IS NOT TRIGERRING");
        for (volatile int i = 0; i < 3500000; ++i);
    }
}

int main(void)
{
    test_adc();
    return 0;
}