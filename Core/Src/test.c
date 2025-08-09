#include "main.h"
#include "uart.h"
#include "io.h"
#include "../external/printf/printf.h"
#include "trace.h"
#include "assert_handler.h"
#include "adc.h"
#include "qre113.h"
#include "line_detect.h"
#include "string.h"

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
        for (volatile int i = 0; i < 3500000; ++i);
    }
}

static void test_qre1113(void)
{
    test_setup();
    trace_init();
    qre113_init();

    while(1)
    {
        struct qre113_voltages volts;
        qre113_read_values(&volts);
        TRACE("ADC VALUE ON LINE DETECT FRONT LEFT: %d\n", volts.front_left);
        TRACE("ADC VALUE ON LINE DETECT FRONT RIGHT: %d\n", volts.front_right);
        TRACE("ADC VALUE ON LINE DETECT BACK LEFT: %d\n", volts.back_left);
        TRACE("ADC VALUE ON LINE DETECT BACK RIGHT: %d\n", volts.back_right);
        HAL_Delay(500);
    }
}

static void test_line_detect(void)
{
    test_setup();
    line_detect_init();
    trace_init();

    while(1)
    {
        line_pos_e line_pos = line_get_positon();
        char position[30];
        switch (line_pos)
        {
            case (LINE_NONE):
                strcpy(position, "No Line");
                break;
            case (LINE_FRONT):
                strcpy(position, "Line Front");
                break;
            case (LINE_BACK):
                strcpy(position, "Line Back");
                break;
            case (LINE_LEFT):
                strcpy(position, "Line Left");
                break;
            case (LINE_RIGHT):
                strcpy(position, "Line Right");
                break;
            case (LINE_FRONT_LEFT):
                strcpy(position, "Line Front Left");
                break;
            case (LINE_FRONT_RIGHT):
                strcpy(position, "Line Front Right");
                break;
            case (LINE_BACK_LEFT):
                strcpy(position, "Line Back Left");
                break;
            case (LINE_BACK_RIGHT):
                strcpy(position, "Line Back Right");
                break;
            case (LINE_DIAGONAL_LEFT):
                strcpy(position, "Line Diag Left");
                break;
            case (LINE_DIAGONAL_RIGHT):
                strcpy(position, "Line Diag Right");
                break;
        }
        TRACE("Line detected on: %s", position);
    }
}

int main(void)
{
    test_qre1113();
    return 0;
}