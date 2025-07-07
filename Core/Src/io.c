/**
  ******************************************************************************
  * @file    io.c
  * @brief   This file provides code for io APIs
  *          of all used io pins.
  ******************************************************************************
**/

#include "io.h"

/********************************************
*                 IO APIs
*********************************************/

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

