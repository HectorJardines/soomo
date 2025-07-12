/**
  ******************************************************************************
  * @file    io.c
  * @brief   This file provides code for io APIs
  *          of all used io pins.
  ******************************************************************************
**/

#include "io.h"
#include <stddef.h>

/********************************************
 * 				STATIC IO ARRAYS
 ********************************************/
static isr_function isr_functions[IO_PORT_CNT][IO_PIN_CNT_PER_PORT] = {
	[IO_PORTA] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	[IO_PORTB] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	[IO_PORTC] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	[IO_PORTD] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
};

/********************************************
*                 IO APIs
*********************************************/
static inline io_port_e IO_GetPort(GPIO_TypeDef *GPIOx)
{
	if (GPIOx == GPIOA)
		return IO_PORTA;
	if (GPIOx == GPIOB)
		return IO_PORTB;
	if (GPIOx == GPIOC)
		return IO_PORTC;
	else
		return IO_PORTD;
}

void IO_Init()
{
	return;
}

void IO_Config(io_handle_t *io_handle)
{
    IO_SetMode(io_handle->GPIOx, io_handle->IO_Confg.PIN_MODE, io_handle->IO_Confg.PIN_NO);
	IO_SetOPType(io_handle->GPIOx, io_handle->IO_Confg.PIN_OPTYPE, io_handle->IO_Confg.PIN_NO);
    IO_SetResistance(io_handle->GPIOx, io_handle->IO_Confg.PIN_RESISTANCE, io_handle->IO_Confg.PIN_NO);
	IO_SetSpeed(io_handle->GPIOx, io_handle->IO_Confg.PIN_SPEED, io_handle->IO_Confg.PIN_NO);
}

/* Set Pin confgs APIs */
void IO_SetResistance(GPIO_TypeDef *gpiox, const io_resistance_e res, const uint8_t io)
{
    switch(res)
    {
      case IO_RES_NOPUPD:
        gpiox->PUPDR &= ~( 0x3U << (io * 2) );
        break;
      case IO_RES_PU:
        gpiox->PUPDR &= ~( 0x3U << (io * 2) );
        gpiox->PUPDR |= ( IO_RES_PU << (io * 2) );
        break;
      case IO_RES_PD:
        gpiox->PUPDR &= ~( 0x3U << (io * 2) );
        gpiox->PUPDR |= ( IO_RES_PD << (io * 2) );
        break;
    }
}
void IO_SetMode(GPIO_TypeDef *gpiox, const io_mode_e mode, const uint8_t io)
{
	switch (mode)
	{
		case IO_MODE_INPUT:
			gpiox->MODER &= ~( 0x3U << (2 * io) );
			break;
		case IO_MODE_OUTPUT:
			gpiox->MODER &= ~( 0x3U << (2 * io) );
			gpiox->MODER |= ( IO_MODE_OUTPUT << (2 * io) );
			break;
		case IO_MODE_ALT_FUN:
			gpiox->MODER &= ~( 0x3U << (2 * io) );
			gpiox->MODER |= ( IO_MODE_ALT_FUN << (2 * io) );
			break;
		case IO_MODE_ANALOG:
			gpiox->MODER &= ~( 0x3U << (2 * io) );
			gpiox->MODER |= ( IO_MODE_ANALOG << (2 * io) );
			break;
	}
}
void IO_SetSpeed(GPIO_TypeDef *gpiox, const io_speed_e speed, const uint8_t io)
{
	switch(speed)
	{
		case IO_SPEED_LOW:
			gpiox->OSPEEDR &= ~( 0x3U << (2 * io) );
			break;
		case IO_SPEED_MEDIUM:
			gpiox->OSPEEDR &= ~( 0x3U << (2 * io) );
			gpiox->OSPEEDR |= ( IO_SPEED_MEDIUM << (2 * io) );
			break;
		case IO_SPEED_FAST:
			gpiox->OSPEEDR &= ~( 0x3U << (2 * io) );
			gpiox->OSPEEDR |= ( 0x2U << (2 * io) );
			break;
		case IO_SPEED_HIGH:
			gpiox->OSPEEDR &= ~( 0x3U << (2 * io) );
			gpiox->OSPEEDR |= ( IO_SPEED_HIGH << (2 * io) );
			break;
	}
}
void IO_SetOPType(GPIO_TypeDef *gpiox, const io_optype_e optype, const uint8_t io)
{
	switch(optype)
	{
		case IO_OPTYPE_PP:
			gpiox->OTYPER &= ~(0x1U << io);
			break;
		case IO_OPTYPE_OD:
			gpiox->OTYPER &= ~(0x1U << io);
			gpiox->OTYPER |= (0x1U << io);
			break;
	}
}

/* Set, Clear, and Toggle Pin APIs */
void IO_SetPin(io_handle_t *io_handle)
{
    // if (io_handle->IO_Confg.PIN_MODE != IO_MODE_OUTPUT)
    //     return;
    io_handle->GPIOx->ODR |= ( 0x1U << io_handle->IO_Confg.PIN_NO );
}

void IO_ClearPin(io_handle_t *io_handle)
{
	// if (io_handle->IO_Confg.PIN_MODE != IO_MODE_OUTPUT)
    //     return;
    io_handle->GPIOx->ODR &= ~( 0x1U << io_handle->IO_Confg.PIN_NO );
}

void IO_TogglePin(io_handle_t *io_handle)
{
	// if (io_handle->IO_Confg.PIN_MODE != IO_MODE_OUTPUT)
    //     return;
    io_handle->GPIOx->ODR ^= ( 0x1U << io_handle->IO_Confg.PIN_NO );
}

/********************************************
*                 IO INTERRUPT APIs
*********************************************/
static void IO_SetInterruptTrigger(uint8_t io, isr_trigger_e trigger)
{
	switch(trigger)
	{
		case IO_INTERRPT_RTFT:
			EXTI->RTSR |= ( 0x1U << io );
			EXTI->FTSR |= ( 0x1U << io );
			break;
		case IO_INTERRUPT_RT:
			EXTI->RTSR |= ( 0x1U << io );
			EXTI->FTSR &= ~( 0x1U << io );
			break;
		case IO_INTERRUPT_FT:
			EXTI->FTSR |= ( 0x1U << io );
			EXTI->RTSR &= ~( 0x1U << io );
			break;
	}
}

static void IO_RegisterISR(uint8_t io, io_port_e port, isr_function isr)
{
	if (isr_functions[port][io] != NULL)
		return;
	isr_functions[port][io] = isr;
}

static void IO_SetEXTILine(uint8_t io, GPIO_TypeDef* gpiox)
{
	const uint8_t EXTI_CRx = io / NO_OF_PINS_IN_CR;
	const uint8_t EXTIx_CR_IDX = ( io % NO_OF_PINS_IN_CR ) * CR_OFFSET_FACTOR;
	if (gpiox == GPIOA)
	{
		SYSCFG->EXTICR[EXTI_CRx] &= ~(SYSCFG_IO_PORTA << EXTIx_CR_IDX);
	}
	else if (gpiox == GPIOB)
	{
		SYSCFG->EXTICR[EXTI_CRx] |= (SYSCFG_IO_PORTB << EXTIx_CR_IDX);
	}
	else if (gpiox == GPIOC)
	{
		SYSCFG->EXTICR[EXTI_CRx] |= (SYSCFG_IO_PORTC << EXTIx_CR_IDX);
	}
	else if (gpiox == GPIOD)
	{
		SYSCFG->EXTICR[EXTI_CRx] |= (SYSCFG_IO_PORTD << EXTIx_CR_IDX);
	}
}

void IO_InitIT(io_handle_t *io_handle, isr_trigger_e trigger, isr_function isr)
{
	
	/* ENABLE SYSCFG PERIPHERAL IN RCC */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* SET SYSCFG CONFIG REG TO APPROPRIATE EXTI LINE */
	IO_SetEXTILine(io_handle->IO_Confg.PIN_NO, io_handle->GPIOx);

	/* SET INTERRUPT TRIGGER AND REGISTER ISR IN ARRAY */
	EXTI->IMR |= (DISABLE_IMR_MASK << io_handle->IO_Confg.PIN_NO);
	IO_SetInterruptTrigger(io_handle->IO_Confg.PIN_NO, trigger);

	IO_RegisterISR(io_handle->IO_Confg.PIN_NO, IO_GetPort(io_handle->GPIOx), isr);
}

void IO_IRQEnableInterrupt(uint16_t EXTI_IRQn)
{
	if (EXTI_IRQn < 32)
	{
		*NVIC_ISER0 |= ( ENABLE << EXTI_IRQn );
	}
	else if (EXTI_IRQn < 64)
	{
		*NVIC_ISER1 |= ( ENABLE << ( EXTI_IRQn % 32 ));
	}
	else if (EXTI_IRQn < 96)
	{
		*NVIC_ISER2 |= ( ENABLE << ( EXTI_IRQn % 64 ));
	}
}

void IO_IRQDisableIT(uint16_t EXTI_IRQn)
{
	if (EXTI_IRQn < 32)
	{
		*NVIC_ICER0 |= ( ENABLE << EXTI_IRQn );
	}
	else if (EXTI_IRQn < 64)
	{
		*NVIC_ICER1 |= ( ENABLE << ( EXTI_IRQn % 32 ));
	}
	else if (EXTI_IRQn < 96)
	{
		*NVIC_ICER2 |= ( ENABLE << ( EXTI_IRQn % 64 ));
	}
}

void IO_SetInterruptPriority(uint16_t EXTI_IRQn, uint8_t IRQ_PR)
{
	uint8_t IPRx = EXTI_IRQn / 4;
	uint8_t IPRx_section = EXTI_IRQn % 4;

	uint8_t shift_amount = ((NVIC_IPRn_OFFSET_MULTIPLIER * IPRx_section) + (TOTAL_NVIC_PR_BITS - NO_PR_BITS_EN));
	*(NVIC_PR_BASE_ADDR + IPRx) |= (IRQ_PR << (shift_amount));
}

static inline uint8_t retrieve_syscfg_exti_port(exti_line_e EXTIx)
{
	uint8_t EXTICRx = EXTIx / 4; // 13 / 4 = 3
	uint8_t EXTI_Shift = ((EXTIx % 4) * 4); // 13 % 4 = 1; 1 * 4 = 4
	uint8_t EXTI_Msk = 0x0F;

	uint8_t current_port = (SYSCFG->EXTICR[EXTICRx] >> EXTI_Shift) & EXTI_Msk;
	return current_port;
}

void EXTI0_IRQHandler(void)
{
	// 1. Get port configuration from SYSCFG_EXTICR[x]
	uint8_t exti_port = retrieve_syscfg_exti_port(EXTI0);
	// 2. Execute ISR from isr_functions array
	isr_functions[exti_port][EXTI0]();
	// 3. Clear Pending Reg bit
	EXTI->PR |= EXTI_PR_PR0_Msk;
}

void EXTI1_IRQHandler(void)
{
	// 1. Get port configuration from SYSCFG_EXTICR[x]
	uint8_t exti_port = retrieve_syscfg_exti_port(EXTI1);
	// 2. Execute ISR from isr_functions array
	isr_functions[exti_port][EXTI1]();
	// 3. Clear Pending Reg bit
	EXTI->PR |= EXTI_PR_PR1_Msk;
}

void EXTI2_IRQHandler(void)
{
	// 1. Get port configuration from SYSCFG_EXTICR[x]
	uint8_t exti_port = retrieve_syscfg_exti_port(EXTI2);
	// 2. Execute ISR from isr_functions array
	isr_functions[exti_port][EXTI2]();
	// 3. Clear Pending Reg bit
	EXTI->PR |= EXTI_PR_PR2_Msk;
}

void EXTI3_IRQHandler(void)
{
	// 1. Get port configuration from SYSCFG_EXTICR[x]
	uint8_t exti_port = retrieve_syscfg_exti_port(EXTI3);
	// 2. Execute ISR from isr_functions array
	isr_functions[exti_port][EXTI3]();
	// 3. Clear Pending Reg bit
	EXTI->PR |= EXTI_PR_PR0_Msk;	
}

void EXTI4_IRQHandler(void)
{
	// 1. Get port configuration from SYSCFG_EXTICR[x]
	uint8_t exti_port = retrieve_syscfg_exti_port(EXTI4);
	// 2. Execute ISR from isr_functions array
	isr_functions[exti_port][EXTI4]();
	// 3. Clear Pending Reg bit
	EXTI->PR |= EXTI_PR_PR0_Msk;
}

static inline uint8_t get_exti_line(void)
{
	for (uint8_t pin_trigger = EXTI_PR_PR5_Pos; pin_trigger < EXTI_PR_PR16_Pos; ++pin_trigger)
	{
		uint8_t check_pr_set = (EXTI->PR >> pin_trigger) & 0x1;
		if (check_pr_set == PRx_SET)
		{
			return pin_trigger;
		} 
	}
	return NO_PRx_SET;
}

void EXTI9_5_IRQHandler(void)
{
	// // 1. Get the pin that triggered Interrupt
	exti_line_e exti_line_trigger = get_exti_line();
	uint8_t exti_port = retrieve_syscfg_exti_port(exti_line_trigger);
	// // 2. Execute ISR from isr_functions array
	if (isr_functions[exti_port][exti_line_trigger] != NULL)
		isr_functions[exti_port][exti_line_trigger]();
	// 3. Clear Pending Reg bit
	EXTI->PR |= (0x1U << exti_line_trigger);
}

void EXTI15_10_IRQHandler(void)
{
	// // 1. Get the pin that triggered Interrupt
	exti_line_e exti_line_trigger = get_exti_line();
	uint8_t exti_port = retrieve_syscfg_exti_port(exti_line_trigger);
	// // 2. Execute ISR from isr_functions array
	if (isr_functions[exti_port][exti_line_trigger] != NULL)
		isr_functions[exti_port][exti_line_trigger]();
	// 3. Clear Pending Reg bit
	EXTI->PR |= (0x1U << exti_line_trigger);
}

