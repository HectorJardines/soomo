#ifndef _IO_H
#define _IO_H

#include "main.h"

/*******************************************
 * 				IO MACROS
 *******************************************/
// #define ENABLE		1
// #define	DISABLE		0

// #define SET_BIT		ENABLE

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


/**************************************
 *            IO ENUMS
 **************************************/

typedef enum
{
  GPIO_UNUSED_1,
  GPIO_UNUSED_2,
  GPIO_UNUSED_3,
  GPIO_UNUSED_4,
  GPIO_UNUSED_5,
  GPIO_UNUSED_6,
  GPIO_UNUSED_7,
  GPIO_UNUSED_8,
  GPIO_UNUSED_9,
  GPIO_UNUSED_10,
  GPIO_UNUSED_11,
  GPIO_UNUSED_12,
  GPIO_UNUSED_13,
  GPIO_UNUSED_14,
  GPIO_UNUSED_15,
  GPIO_UNUSED_16,
  GPIO_UNUSED_17,
  GPIO_UNUSED_18,
  GPIO_UNUSED_19,
  GPIO_UNUSED_20,
  GPIO_UNUSED_21,
  GPIO_UNUSED_22,
  GPIO_UNUSED_23,
  GPIO_UNUSED_24,
  GPIO_UNUSED_25,
  GPIO_UNUSED_26,
  GPIO_UNUSED_27,
  GPIO_UNUSED_28,
  GPIO_UNUSED_29,
  GPIO_UNUSED_30,
  GPIO_UNUSED_31,
  GPIO_UNUSED_32,
  GPIO_UNUSED_33,
  GPIO_UNUSED_34,
  GPIO_UNUSED_35,
  GPIO_UNUSED_36,
  GPIO_UNUSED_37,
  GPIO_UNUSED_38,
  GPIO_UNUSED_39,
  GPIO_UNUSED_40,
  GPIO_UNUSED_41,
  GPIO_UNUSED_42,
  GPIO_UNUSED_43,
  GPIO_UNUSED_44,
  GPIO_UNUSED_45,
  GPIO_UNUSED_46,
  GPIO_UNUSED_47,
  GPIO_UNUSED_48,
  GPIO_UNUSED_49,
  GPIO_UNUSED_50
} generic_io_e;

typedef enum
{
  // TODO: DEFINE IO PINS WITH NAMES TO BE USED FOR SENSORS,
  //       MOTOR DRIVERS, TIMERS, UART, ETC.
  IO_MOTOR_PWM
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

/* IO CONFIG STRUCT */
typedef struct
{
  uint8_t PIN_NO;
  io_mode_e PIN_MODE;                 /*<! PIN MODE OPTIONS FROM @io_mode_e >*/
  io_speed_e PIN_SPEED;               /*<! PIN SPEED OPTIONS FROM @io_speed_e >*/
  io_optype_e PIN_OPTYPE;             /*<! PIN OUTPUT TYPE OPTIONS FROM @io_optype_e >*/
  io_resistance_e PIN_RESISTANCE;     /*<! PIN RESISTOR OPTIONS FROM @io_resistance_e >*/
}io_config_t;

typedef struct
{
  GPIO_TypeDef *GPIOx;
  io_config_t  IO_Confg;
} io_handle_t;


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
void IO_SetPin(io_handle_t *io_handle);
void IO_ClearPin(io_handle_t *io_handle);
void IO_TogglePin(io_handle_t *io_handle);

#endif