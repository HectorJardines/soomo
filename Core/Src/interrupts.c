#include "interrupts.h"

void IT_IRQEnableInterrupt(uint16_t EXTI_IRQn)
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

void IT_IRQDisableIT(uint16_t EXTI_IRQn)
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

void IT_SetInterruptPriority(uint16_t EXTI_IRQn, uint8_t IRQ_PR)
{
	uint8_t IPRx = EXTI_IRQn / 4;
	uint8_t IPRx_section = EXTI_IRQn % 4;

	uint8_t shift_amount = ((NVIC_IPRn_OFFSET_MULTIPLIER * IPRx_section) + (TOTAL_NVIC_PR_BITS - NO_PR_BITS_EN));
	*(NVIC_PR_BASE_ADDR + IPRx) |= (IRQ_PR << (shift_amount));
}