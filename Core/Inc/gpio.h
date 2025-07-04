/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

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
  IO_OPTYPE_NOPUPD,
  IO_OPTYPE_PU,
  IO_OPTYPE_PD
} io_resistance_e;

/* IO CONFIG STRUCT */
typedef struct
{
  io_mode_e PIN_MODE;                 /*<! PIN MODE OPTIONS FROM @io_mode_e >*/
  io_speed_e PIN_SPEED;               /*<! PIN SPEED OPTIONS FROM @io_speed_e >*/
  io_optype_e PIN_OPTYPE;             /*<! PIN OUTPUT TYPE OPTIONS FROM @io_optype_e >*/
  io_resistance_e PIN_RESISTANCE;     /*<! PIN RESISTOR OPTIONS FROM @io_resistance_e >*/
}io_config_t;

/********************************************
*                 IO APIs
*********************************************/

void IO_Init();
void IO_Config(io_e io, const io_config_t *io_confg);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

