#ifndef _IO_H
#define _IO_H

#include "main.h"

/*******************************************
 * 				IO MACROS
 *******************************************/
// #define ENABLE		1
// #define	DISABLE		0

// #define SET_BIT		ENABLE

#define NO_OF_PINS_PER_REG    8
#define NO_OF_AFR_BITS        4

#define IO_PIN_0        0U
#define IO_PIN_1        1U
#define IO_PIN_2        2U
#define IO_PIN_3        3U
#define IO_PIN_4        4U
#define IO_PIN_5        5U
#define IO_PIN_6        6U
#define IO_PIN_7        7U
#define IO_PIN_8        8U
#define IO_PIN_9        9U
#define IO_PIN_10       10U
#define IO_PIN_11       11U
#define IO_PIN_12       12U
#define IO_PIN_13       13U
#define IO_PIN_14       14U
#define IO_PIN_15       15U

#define NO_OF_PINS_IN_CR  4U
#define CR_OFFSET_FACTOR  4U

#define DISABLE_IMR_MASK  1U

#define IO_PORT_CNT           3U
#define IO_PIN_CNT_PER_PORT   16U

#define PRx_SET     1U
#define NO_PRx_SET  23U

/*******************************************
 * 				       NVIC MACROS
 *******************************************/
#define NVIC_ISER0      ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1      ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2      ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3      ((volatile uint32_t*)0xE000E10C)

#define NVIC_ICER0      ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1      ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2      ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3      ((volatile uint32_t*)0xE000E18C)

#define NVIC_PR_BASE_ADDR		((volatile uint32_t*)0xE000E400)

#define TOTAL_NVIC_PR_BITS            8U
#define NO_PR_BITS_EN                 4U
#define NVIC_IPRn_OFFSET_MULTIPLIER   8U

/**************************************
 *            IO ENUMS
 **************************************/

typedef enum
{
  // PORTA PINS
  IO_A0, IO_A1, IO_A2, IO_A3, 
  IO_A4, IO_A5, IO_A6, IO_A7, 
  IO_A8, IO_A9, IO_A10, IO_A11, 
  IO_A12, IO_A13, IO_A14, IO_A15,
  
  // PORTB PINS
  IO_B0, IO_B1, IO_B2, IO_B3,
  IO_B4, IO_B5, IO_B6, IO_B7,
  IO_B8, IO_B9, IO_B10, IO_B11,
  IO_B12, IO_B13, IO_B14, IO_B15,

  //PORTC PINS
  IO_C0, IO_C1, IO_C2, IO_C3,
  IO_C4, IO_C5, IO_C6, IO_C7,
  IO_C8, IO_C9, IO_C10, IO_C11,
  IO_C12, IO_C13, IO_C14, IO_C15
} generic_io_e;

typedef enum
{
  // TODO: DEFINE IO PINS WITH NAMES TO BE USED FOR SENSORS,
  //       MOTOR DRIVERS, TIMERS, UART, ETC.
  IO_PWM_MOTOR_LEFT = IO_A0,
  IO_PWM_MOTOR_RIGHT = IO_A1,
  IO_UART_TX = IO_A2,
  IO_UART_RX = IO_A3,
  IO_UNUSED_2 = IO_A4,
  IO_TEST_LED = IO_A5,
  IO_UNUSED_4 = IO_A6,
  IO_UNUSED_5 = IO_A7,
  IO_UNUSED_6 = IO_A8,
  IO_UNUSED_7 = IO_A9,
  IO_UNUSED_8 = IO_A10,
  IO_UNUSED_9 = IO_A11,
  IO_UNUSED_10 = IO_A12,
  IO_UNUSED_11 = IO_A13,
  IO_UNUSED_12 = IO_A14,
  IO_UNUSED_13 = IO_A15,

  IO_UNUSED_14 = IO_B0,
  IO_MOTORS_RIGHT_CC2 = IO_B1,
  IO_MOTORS_RIGHT_CC1 = IO_B2,
  IO_UNUSED_17 = IO_B3,
  IO_UNUSED_18 = IO_B4,
  IO_UNUSED_19 = IO_B5,
  IO_UNUSED_20 = IO_B6,
  IO_UNUSED_21 = IO_B7,
  IO_UNUSED_22 = IO_B8,
  IO_UNUSED_23 = IO_B9,
  IO_UNUSED_24 = IO_B10,
  IO_UNUSED_25 = IO_B11,
  IO_UNUSED_26 = IO_B12,
  IO_UNUSED_27 = IO_B13,
  IO_UNUSED_28 = IO_B14,
  IO_UNUSED_29 = IO_B15,

  IO_UNUSED_30 = IO_C0,
  IO_UNUSED_31 = IO_C1,
  IO_MOTORS_LEFT_CC2 = IO_C2,
  IO_MOTORS_LEFT_CC1 = IO_C3,
  IO_UNUSED_34 = IO_C4,
  IO_UNUSED_35 = IO_C5,
  IO_UNUSED_36 = IO_C6,
  IO_UNUSED_37 = IO_C7,
  IO_UNUSED_38 = IO_C8,
  IO_UNUSED_39 = IO_C9,
  IO_UNUSED_40 = IO_C10,
  IO_UNUSED_41 = IO_C11,
  IO_UNUSED_42 = IO_C12,
  IO_UNUSED_43 = IO_C13,
  IO_UNUSED_44 = IO_C14,
  IO_UNUSED_45 = IO_C15,
} io_e;

typedef enum
{
	IO_MODE_INPUT,
	IO_MODE_OUTPUT,
	IO_MODE_ALT_FUN,
	IO_MODE_ANALOG
}io_mode_e;

typedef enum
{
	IO_SPEED_LOW,
	IO_SPEED_MEDIUM,
	IO_SPEED_FAST,
	IO_SPEED_HIGH
} io_speed_e;

typedef enum
{
	IO_OPTYPE_PP,
	IO_OPTYPE_OD,
} io_optype_e;

typedef enum
{
    IO_RES_NOPUPD,
    IO_RES_PU,
    IO_RES_PD
} io_resistance_e;

typedef enum
{
	IO_INTERRPT_RTFT,
	IO_INTERRUPT_RT,
	IO_INTERRUPT_FT
} isr_trigger_e;

typedef enum
{
	SYSCFG_IO_PORTA,
	SYSCFG_IO_PORTB,
	SYSCFG_IO_PORTC,
	SYSCFG_IO_PORTD
} syscfg_io_port_e;

typedef enum
{
	IO_PORTA,
	IO_PORTB,
	IO_PORTC,
	IO_PORTD
} io_port_e;

typedef enum
{
  IO_ALT_FUN_MODE0,
  IO_ALT_FUN_MODE1,
  IO_ALT_FUN_MODE2,
  IO_ALT_FUN_MODE3,
  IO_ALT_FUN_MODE4,
  IO_ALT_FUN_MODE5,
  IO_ALT_FUN_MODE6,
  IO_ALT_FUN_MODE7,
  IO_ALT_FUN_MODE8,
  IO_ALT_FUN_MODE9,
  IO_ALT_FUN_MODE10,
  IO_ALT_FUN_MODE11,
  IO_ALT_FUN_MODE12,
  IO_ALT_FUN_MODE13,
  IO_ALT_FUN_MODE14,
  IO_ALT_FUN_MODE15
} io_mode_alt_fun;

/* IO CONFIG STRUCT */
typedef struct
{
	uint8_t PIN_NO;
	io_mode_e PIN_MODE;                 /*<! PIN MODE OPTIONS FROM @io_mode_e >*/
	io_speed_e PIN_SPEED;               /*<! PIN SPEED OPTIONS FROM @io_speed_e >*/
	io_optype_e PIN_OPTYPE;             /*<! PIN OUTPUT TYPE OPTIONS FROM @io_optype_e >*/
	io_resistance_e PIN_RESISTANCE;     /*<! PIN RESISTOR OPTIONS FROM @io_resistance_e >*/
  io_mode_alt_fun PIN_ALT_FUN_MODE;
}io_config_t;

typedef struct
{
	GPIO_TypeDef *GPIOx;
	io_config_t  IO_Confg;
} io_handle_t;

typedef enum
{
	EXTI0, EXTI1, EXTI2, EXTI3,
	EXTI4, EXTI5, EXTI6, EXTI7,
	EXTI8, EXTI9, EXTI10, EXTI11,
	EXTI12, EXTI13, EXTI14, EXTI15
}exti_line_e;

typedef enum
{
	EXTI0_IRQ_NO = 6,
	EXTI1_IRQ_NO,
	EXTI2_IRQ_NO,
	EXTI3_IRQ_NO,
	EXTI4_IRQ_NO,
	EXTI9_5_IRQ_NO = 23,
	EXTI15_10_IRQ_NO = 40
}exti_irqn_e;

typedef enum
{
  LOW,
  HIGH
}io_value_e;

/********************************************
*                 IO APIs
*********************************************/

void IO_Init();
void IO_Config(io_handle_t *io_handle);

/* Set Pin confgs APIs */
void IO_SetResistance(GPIO_TypeDef *gpiox, const io_resistance_e res, const uint8_t io);
void IO_SetMode(GPIO_TypeDef *gpiox, const io_mode_e mode, const uint8_t io);
void IO_SetSpeed(GPIO_TypeDef *gpiox, const io_speed_e speed, const uint8_t io);
void IO_SetOPType(GPIO_TypeDef *gpiox, const io_optype_e optype, const uint8_t io);

/* Set, Clear, and Toggle Pin APIs */
void io_write_pin(io_e io, io_value_e value);
void IO_SetPin(io_handle_t *io_handle);
void IO_ClearPin(io_handle_t *io_handle);
void IO_TogglePin(io_handle_t *io_handle);

/* IO Interrupt APIs */
typedef void (*isr_function)(void);

void IO_InitIT(io_handle_t *io_handle, isr_trigger_e trigger, isr_function isr);
void IO_DeInitIT(uint8_t io);

void IO_IRQEnableInterrupt(uint16_t EXTI_IRQn);
void IO_IRQDisableIT(uint16_t EXTI_IRQn);
void IO_SetInterruptPriority(uint16_t EXTI_IRQn, uint8_t IRQ_PR);

/******************************************
*           IO ISR Functions
*******************************************/
#endif