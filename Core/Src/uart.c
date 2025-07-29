#include "uart.h"
#include "ring_buffer.h"

#define UART_BUFFER_SIZE    (16U)
static uint8_t buffer[UART_BUFFER_SIZE];
static struct ring_buffer tx_buf = {.buf = buffer, .buf_size = sizeof(buffer), .elem_size = sizeof(uint8_t), .head = 0, .tail = 0};
static USART_Handle_t usart2_h = {{.USART_Mode = 0, .USART_ParityCtrl = USART_PARITY_CRL_DI, .USART_NO_OF_STOP_BITS = USART_STOP_BITS1, .USART_WordLen = USART_WORD_LEN8, .USART_BaudRate = 115200, .USART_HWFlowCtrl = USART_NO_HWFlowCtrl}, .USARTx = USART2};

/********************************************************
 *                     USART CONFG APIs
 ********************************************************/

static inline void usart_set_mode(USART_TypeDef *pUSARTx)
{
    pUSARTx->CR1 |= USART_CR1_TE | USART_CR1_RE;
}

static inline void usart_set_baud_rate(USART_TypeDef *pUSARTx, uint32_t BaudRate)
{
    // Tx/Rx baud = fCLK / (8 * ( 2 - OVER8 ) * USARTDIV)
    // USARTDIV = fCLK / ( 8 * (2 - OVER8 ) * baud)

    uint32_t clkRate;
    uint32_t usart_div;
    uint32_t mantissa;
    uint32_t fraction;
    uint32_t BRR_Value;

    if (pUSARTx == USART1 || pUSARTx == USART6)
        clkRate = HAL_RCC_GetPCLK2Freq();
    else
        clkRate = HAL_RCC_GetPCLK1Freq();

    // OVER8 = 1, OVERSAMPLING by 8
    if (pUSARTx->CR1 & (USART_CR1_OVER8_Msk))
        // (fCLK / (8 * BR)) * 100 (to scale to integer)
        usart_div =  ((25 * clkRate) / (2 * BaudRate));
    else
        usart_div = ((25 * clkRate) / (4 * BaudRate));

    mantissa = (usart_div / 100);
    // 100.75

    BRR_Value = (mantissa << USART_BRR_DIV_Mantissa_Pos);
    fraction = (usart_div - (mantissa * 100));

    // TODO: FIX BRR_VALUE COMPUTATION WITH FRACTIONAL PART
    if (pUSARTx->CR1 & (USART_CR1_OVER8_Msk))
        // Multiply by 8 
        fraction = (((fraction * 8) + 50) / 100) & ((uint8_t) 0x7);
    else
        fraction = (((fraction * 16) + 50) / 100) & ((uint8_t) 0x0F);

    BRR_Value |= fraction;
    pUSARTx->BRR = BRR_Value;
}

static inline void usart_set_parity_ctrl(USART_TypeDef *pUSARTx, uint8_t ParityEnOrDi)
{
    switch (ParityEnOrDi)
    {
        case USART_PARITY_CRL_EVEN_EN:
            pUSARTx->CR1 |= USART_CR1_PCE;
            pUSARTx->CR1 &= ~(USART_CR1_PS);
            break;
        case USART_PARITY_CRL_ODD_EN:
            pUSARTx->CR1 |= USART_CR1_PCE | USART_CR1_PS;
            break;
        case USART_PARITY_CRL_DI:
            pUSARTx->CR1 &= ~(USART_CR1_PCE);
            break;
    }
}

static inline void usart_set_word_length(USART_TypeDef *pUSARTx, uint8_t WordLen)
{
    if (WordLen == USART_WORD_LEN9)
        pUSARTx->CR1 |= USART_CR1_M;
    else if (WordLen == USART_WORD_LEN8)
        pUSARTx->CR1 &= ~(USART_CR1_M);
}

static inline void usart_set_hwflow_ctrl(USART_TypeDef *pUSARTx, uint8_t HWFlowCtrl)
{
    switch (HWFlowCtrl)
    {
        case USART_RTS_CTS:
            pUSARTx->CR3 |= USART_CR3_CTSE_Msk | USART_CR3_RTSE_Msk;
            break;
        case USART_RTS_ONLY:
            pUSARTx->CR3 |= USART_CR3_RTSE_Msk;
            break;
        case USART_CTS_ONLY:
            pUSARTx->CR3 |= USART_CR3_CTSE_Msk;
            break;
        case USART_NO_HWFlowCtrl:
            pUSARTx->CR3 &= ~(USART_CR3_RTSE_Msk);
            pUSARTx->CR3 &= ~(USART_CR3_CTSE_Msk);
            break;
    }
}

static inline void usart_set_stop_bits(USART_TypeDef *pUSARTx, uint8_t StopBits)
{
    switch (StopBits)
    {
        case USART_STOP_BITS1:
            pUSARTx->CR2 &= ~( 0x1 << USART_CR2_STOP_Pos );
            break;
        case USART_STOP_BITS0_5:
            pUSARTx->CR2 |= ( USART_STOP_BITS0_5 << USART_CR2_STOP_Pos );
            break;
        case USART_STOP_BITS2:
            pUSARTx->CR2 |= ( USART_STOP_BITS2 << USART_CR2_STOP_Pos );
            break;
        case USART_STOP_BITS1_5:
            pUSARTx->CR2 |= ( USART_STOP_BITS1_5 << USART_CR2_STOP_Pos );
            break;
    }
}

void usart_enable_peripheral(USART_TypeDef *pUSARTx)
{
    pUSARTx->CR1 |= (USART_CR1_UE);
}

void usart_configure(USART_Handle_t *USART_Handle)
{
    usart_enable_peripheral(USART_Handle->USARTx);

    usart_set_mode(USART_Handle->USARTx);
    usart_set_baud_rate(USART_Handle->USARTx, USART_Handle->USART_Confg.USART_BaudRate);
    usart_set_parity_ctrl(USART_Handle->USARTx, USART_Handle->USART_Confg.USART_ParityCtrl);
    usart_set_word_length(USART_Handle->USARTx, USART_Handle->USART_Confg.USART_WordLen);
    usart_set_hwflow_ctrl(USART_Handle->USARTx, USART_Handle->USART_Confg.USART_HWFlowCtrl);
    usart_set_stop_bits(USART_Handle->USARTx, USART_Handle->USART_Confg.USART_NO_OF_STOP_BITS);
}

/********************************************************
 *                     USART IT APIs
 ********************************************************/

static void usart_tx_interrupt_enable(void)
{
    USART2->CR1 |= USART_CR1_TXEIE;
}

static void usart_tx_interrupt_disable(void)
{
    USART2->CR1 &= ~(USART_CR1_TXEIE);
}

static void usart_start_tx(void)
{
    if (!ring_buffer_empty(&tx_buf))
    {
        char c = 0;
        ring_buffer_peek_tail(&tx_buf, &c);
        USART2->DR = c;
    }
}

void _putchar(char c)
{
    if (c == '\n')
        _putchar('\r');
    // 1. Ensure that transmission is not already happening
    while(ring_buffer_full(&tx_buf));

    usart_tx_interrupt_disable();
    const bool tx_ongoing = !(ring_buffer_empty(&tx_buf));
    ring_buffer_push(&tx_buf, &c);

    if (!tx_ongoing)
        usart_start_tx();

    usart_tx_interrupt_enable();
}

void USART2_IRQHandler(void)
{
    /************************ USART TXE IT ***********************/
    ring_buffer_pop(&tx_buf);
    if (!ring_buffer_empty(&tx_buf))
        usart_start_tx();

    if (ring_buffer_empty(&tx_buf))
        usart_tx_interrupt_disable();
}

static uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx, uint8_t flag)
{
    if (pUSARTx->SR & flag)
    {
        return 1;
    }
    return 0;
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	// uint16_t *pdata;

   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->USARTx, USART_SR_TXE));

        //This is 8bit data transfer
        pUSARTHandle->USARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

        //Implement the code to increment the buffer address
        pTxBuffer++;
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->USARTx, USART_SR_TC));
}

static void usart_irq_priority_set(uint16_t EXTI_IRQn, uint8_t IRQ_PR)
{
    uint8_t IPRx = EXTI_IRQn / 4;
	uint8_t IPRx_section = EXTI_IRQn % 4;

	uint8_t shift_amount = ((NVIC_IPRn_OFFSET_MULTIPLIER * IPRx_section) + (TOTAL_NVIC_PR_BITS - NO_PR_BITS_EN));
	*(NVIC_PR_BASE_ADDR + IPRx) |= (IRQ_PR << (shift_amount));
}

void USART_IRQEnableInterrupt(uint16_t EXTI_IRQn)
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

void USART_IRQDisableIT(uint16_t EXTI_IRQn)
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

static inline void usart_peripheral_control_enable(void)
{
    // Only concerned with USART2, could abstract to include all USART peripherals
    RCC->APB1ENR |= (RCC_APB1ENR_USART2EN);
}

static inline void usart_peripheral_control_disable(void)
{
    // Only concerned with USART2, could abstract to include all USART peripherals
    RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN);
}

void usart_init(void)
{
    usart_peripheral_control_enable();
    usart_configure(&usart2_h);
    
    usart_irq_priority_set(USART2_IRQ_NO, 2);
    USART_IRQEnableInterrupt(USART2_IRQ_NO);
}
