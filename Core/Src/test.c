#include "main.h"
#include "uart.h"
#include "io.h"
#include "../external/printf/printf.h"
#include "trace.h"

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

int main(void)
{
    test_trace();
    return 0;
}