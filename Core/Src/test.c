#include "main.h"
#include "uart.h"
#include "io.h"

void test_setup(void)
{
    SystemClock_Config();
}

void test_io_led(void)
{
    test_setup();
    IO_Init();
    while (1)
    {
        HAL_Delay(500);
        GPIOA->ODR ^= (0x1U << GPIO_ODR_OD5_Pos);
    }
}

void test_uart(void)
{
    test_setup();
    IO_Init();
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

int main(void)
{
    test_uart();
    return 0;
}