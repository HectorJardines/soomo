#ifndef _INTERRUPTS_H
#define _INTERRUPTS_H
#include <stdint.h>
#include "main.h"

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

/******************************************
 *              NVIC APIs
 ******************************************/

void IT_IRQEnableInterrupt(uint16_t EXTI_IRQn);
void IT_IRQDisableIT(uint16_t EXTI_IRQn);
void IT_SetInterruptPriority(uint16_t EXTI_IRQn, uint8_t IRQ_PR);

#endif