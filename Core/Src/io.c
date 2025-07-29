/**
  ******************************************************************************
  * @file    io.c
  * @brief   This file provides code for io APIs
  *          of all used io pins.
  ******************************************************************************
**/

#include "io.h"
#include <stddef.h>
#include "defines.h"

#define ANALOG_IO_CONFG(GPIOX, IO_PIN_NO)	{(GPIOX), {(IO_PIN_NO), IO_MODE_ANALOG, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD}}
// #define UNUSED_IO_CONFG(GPIOX, IO_PIN_NO) 	{(GPIOX), {(IO_PIN_NO), IO_MODE_OUTPUT, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_PD}}
#define UNUSED_IO_CONFG(GPIOX, IO_PIN_NO)	{NULL, {16}}
/********************************************
 * 				STATIC IO ARRAYS
 ********************************************/
static isr_function isr_functions[IO_PORT_CNT][IO_PIN_CNT_PER_PORT] = {
	[IO_PORTA] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	[IO_PORTB] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	[IO_PORTC] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
};

static io_handle_t io_confgs[IO_PORT_CNT * IO_PIN_CNT_PER_PORT] = {
	//MOTOR PWM
	[IO_PWM_MOTOR_LEFT] = {GPIOA, {IO_PIN_0, IO_MODE_ALT_FUN, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD, IO_ALT_FUN_MODE1}},
	[IO_PWM_MOTOR_RIGHT] = {GPIOA, {IO_PIN_1, IO_MODE_ALT_FUN, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD, IO_ALT_FUN_MODE1}},

	//UART TX/RX
	[IO_UART_TX] = {GPIOA, {IO_PIN_2, IO_MODE_ALT_FUN, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD, IO_ALT_FUN_MODE7}},
	[IO_UART_RX] = {GPIOA, {IO_PIN_3, IO_MODE_ALT_FUN, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD, IO_ALT_FUN_MODE7}},

	//OUTPUT
	[IO_TEST_LED] = {GPIOA, {IO_PIN_5, IO_MODE_OUTPUT, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD}},
	
	//MOTOR CONTROL
	[IO_MOTORS_LEFT_CC2] = {GPIOB, {IO_PIN_1, IO_MODE_OUTPUT, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD}},
	[IO_MOTORS_LEFT_CC2] = {GPIOB, {IO_PIN_2, IO_MODE_OUTPUT, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD}},

	[IO_MOTORS_RIGHT_CC2] = {GPIOC, {IO_PIN_2, IO_MODE_OUTPUT, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD}},
	[IO_MOTORS_RIGHT_CC2] = {GPIOC, {IO_PIN_3, IO_MODE_OUTPUT, IO_SPEED_FAST, IO_OPTYPE_PP, IO_RES_NOPUPD}},

	// ADC INPUT PINS
	[IO_LINE_DETECT_FRONT_LEFT] = ANALOG_IO_CONFG(GPIOA, IO_PIN_4),
	[IO_LINE_DETECT_BACK_LEFT] = ANALOG_IO_CONFG(GPIOB, IO_PIN_0),

	[IO_LINE_DETECT_FRONT_RIGHT] = ANALOG_IO_CONFG(GPIOA, IO_PIN_6),
	[IO_LINE_DETECT_BACK_RIGHT] = ANALOG_IO_CONFG(GPIOA, IO_PIN_7),

	//UNUSED PINS PORTA
	[IO_UNUSED_6] = UNUSED_IO_CONFG(GPIOA, IO_PIN_8), [IO_UNUSED_7] = UNUSED_IO_CONFG(GPIOA, IO_PIN_9), 
	[IO_UNUSED_8] = UNUSED_IO_CONFG(GPIOA, IO_PIN_10), [IO_UNUSED_9] = UNUSED_IO_CONFG(GPIOA, IO_PIN_11), [IO_UNUSED_10] = UNUSED_IO_CONFG(GPIOA, IO_PIN_12),
	[IO_UNUSED_11] = UNUSED_IO_CONFG(GPIOA, IO_PIN_13), [IO_UNUSED_12] = UNUSED_IO_CONFG(GPIOA, IO_PIN_14), [IO_UNUSED_13] = UNUSED_IO_CONFG(GPIOA, IO_PIN_15),

	//UNUSED PINS PORTB
	[IO_UNUSED_17] = UNUSED_IO_CONFG(GPIOB, IO_PIN_3), [IO_UNUSED_18] = UNUSED_IO_CONFG(GPIOB, IO_PIN_4), [IO_UNUSED_19] = UNUSED_IO_CONFG(GPIOB, IO_PIN_5), 
	[IO_UNUSED_20] = UNUSED_IO_CONFG(GPIOB, IO_PIN_6), [IO_UNUSED_21] = UNUSED_IO_CONFG(GPIOB, IO_PIN_7), [IO_UNUSED_22] = UNUSED_IO_CONFG(GPIOB, IO_PIN_8), [IO_UNUSED_23] = UNUSED_IO_CONFG(GPIOB, IO_PIN_9), 
	[IO_UNUSED_24] = UNUSED_IO_CONFG(GPIOB, IO_PIN_10), [IO_UNUSED_25] = UNUSED_IO_CONFG(GPIOB, IO_PIN_11), [IO_UNUSED_26] = UNUSED_IO_CONFG(GPIOB, IO_PIN_12), [IO_UNUSED_27] = UNUSED_IO_CONFG(GPIOB, IO_PIN_13), 
	[IO_UNUSED_28] = UNUSED_IO_CONFG(GPIOB, IO_PIN_14), [IO_UNUSED_29] = UNUSED_IO_CONFG(GPIOB, IO_PIN_15),

	//UNUSED PINS PORTC
	[IO_UNUSED_30] = UNUSED_IO_CONFG(GPIOC, IO_PIN_0), [IO_UNUSED_31] = UNUSED_IO_CONFG(GPIOC, IO_PIN_1), [IO_UNUSED_34] = UNUSED_IO_CONFG(GPIOC, IO_PIN_4), [IO_UNUSED_35] = UNUSED_IO_CONFG(GPIOC, IO_PIN_5), 
	[IO_UNUSED_36] = UNUSED_IO_CONFG(GPIOC, IO_PIN_6), [IO_UNUSED_37] = UNUSED_IO_CONFG(GPIOC, IO_PIN_7), [IO_UNUSED_38] = UNUSED_IO_CONFG(GPIOC, IO_PIN_8), [IO_UNUSED_39] = UNUSED_IO_CONFG(GPIOC, IO_PIN_9), 
	[IO_UNUSED_40] = UNUSED_IO_CONFG(GPIOC, IO_PIN_10), [IO_UNUSED_41] = UNUSED_IO_CONFG(GPIOC, IO_PIN_11), [IO_UNUSED_42] = UNUSED_IO_CONFG(GPIOC, IO_PIN_12), [IO_UNUSED_43] = UNUSED_IO_CONFG(GPIOC, IO_PIN_13), 
	[IO_UNUSED_44] = UNUSED_IO_CONFG(GPIOC, IO_PIN_14), [IO_UNUSED_45] = UNUSED_IO_CONFG(GPIOC, IO_PIN_15)
};

static const io_e adc_pins_arr[] =
{
	IO_LINE_DETECT_FRONT_LEFT,
	IO_LINE_DETECT_BACK_LEFT,
	IO_LINE_DETECT_FRONT_RIGHT,
	IO_LINE_DETECT_BACK_RIGHT
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
static bool compare_io_unused(const io_handle_t *io_h)
{
	return (io_h->GPIOx == NULL) && (io_h->IO_Confg.PIN_NO == 16);
}

void IO_Init()
{
	for (generic_io_e io = (generic_io_e)IO_A0; io < ARRAY_SIZE(io_confgs); io++)
		if (!compare_io_unused(&io_confgs[io]))
			IO_Config(&io_confgs[io]);
	return;
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
static void IO_SetAltFunMode(GPIO_TypeDef *gpiox, const io_mode_alt_fun alt_fun_mode, const uint8_t io)
{
	// Retrieve index of Alt Fun register to be used
	uint8_t AFR_Index = io / NO_OF_PINS_PER_REG;
	gpiox->AFR[AFR_Index] |= (alt_fun_mode << (io * NO_OF_AFR_BITS));
}

static void io_peripheral_control(GPIO_TypeDef *gpiox, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (gpiox == GPIOA)
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		if (gpiox == GPIOB)
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
		if (gpiox == GPIOC)
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	}
	else
	{
		if (gpiox == GPIOA)
			RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN);
		if (gpiox == GPIOB)
			RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN);
		if (gpiox == GPIOC)
			RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOCEN);
	}
}

void IO_Config(io_handle_t *io_handle)
{
    IO_SetMode(io_handle->GPIOx, io_handle->IO_Confg.PIN_MODE, io_handle->IO_Confg.PIN_NO);
	IO_SetOPType(io_handle->GPIOx, io_handle->IO_Confg.PIN_OPTYPE, io_handle->IO_Confg.PIN_NO);
    IO_SetResistance(io_handle->GPIOx, io_handle->IO_Confg.PIN_RESISTANCE, io_handle->IO_Confg.PIN_NO);
	IO_SetSpeed(io_handle->GPIOx, io_handle->IO_Confg.PIN_SPEED, io_handle->IO_Confg.PIN_NO);
	if (io_handle->IO_Confg.PIN_MODE == IO_MODE_ALT_FUN)
		IO_SetAltFunMode(io_handle->GPIOx, io_handle->IO_Confg.PIN_ALT_FUN_MODE, io_handle->IO_Confg.PIN_NO);
	
	io_peripheral_control(io_handle->GPIOx, ENABLE);
}

/* Set, Clear, and Toggle Pin APIs */

void io_write_pin(io_e io, io_value_e value)
{
	if (value == HIGH)
		io_confgs[io].GPIOx->ODR |= (HIGH << io_confgs[io].IO_Confg.PIN_NO);
	else if (value == LOW)
		io_confgs[io].GPIOx->ODR &= ~(HIGH << io_confgs[io].IO_Confg.PIN_NO);
}

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

/********************************************
*                 IO EXTI IRQ Handlers
*********************************************/

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

