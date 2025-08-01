#include "assert_handler.h"
#include "../external/printf/printf.h"
#include "main.h"
#include "uart.h"

#define BREAKPOINT __asm volatile ("bkpt #0")
// string + program counter + null term
#define ASSERT_STRING_MAX_SIZE      (15u + 8u + 1u)

static void assert_trace(uint16_t pc)
{
    // CONFIG TX PIN 
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
    GPIOA->MODER |= (MODE_AF << (2 * 2));
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2);
    GPIOA->AFR[0] |= (GPIO_AF7_USART2 << (2 * 2));
    // USART INIT FOR ASSERT
    usart_assert_init();

    char assert_string[ASSERT_STRING_MAX_SIZE];
    snprintf(assert_string, sizeof(assert_string), "ASSERT 0x%x\n", pc);
    usart_trace_assert(assert_string);
}

static void assert_blink_led(void)
{
    // CONFIG TEST LED IN CASE ASSERT CALLED BEFORE IT WAS CONFIG
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
    GPIOA->MODER |= (MODE_OUTPUT << (5 * 2));
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);

    while(1)
    {
        for (volatile int i = 0; i < 300000; i++);
        GPIOA->ODR ^= (GPIO_ODR_OD5);
    }
}

void assert_handler(uint16_t pc)
{
    BREAKPOINT;
    assert_trace(pc);
    assert_blink_led();
}