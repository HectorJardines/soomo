#include "uart.h"

void usart_init(USART_Handle_t *USART_Handle)
{
    usart_configure(USART_Handle);
}

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

static void clear_tx_interrupt(USART_TypeDef *pUSARTx)
{
    uint8_t dummy_write = 0xFF;
    pUSARTx->DR |= dummy_write;
    (void) dummy_write;
}

static void usart_tx_interrupt_enable(USART_TypeDef *pUSARTx)
{
    pUSARTx->CR1 |= USART_CR1_TXEIE;
}

static void usart_tx_interrupt_disable(USART_TypeDef *pUSARTx)
{
    pUSARTx->CR1 &= ~(USART_CR1_TXEIE);
}

static void usart_tc_interrupt_enable(USART_TypeDef *pUSART)
{
    pUSART->CR1 |= USART_CR1_TCIE;
}

static void usart_clear_tc(USART_TypeDef *pUSARTx)
{
    uint8_t dummy_read, dummy_write;
    dummy_write = 0x00;

    dummy_read = pUSARTx->SR;
    pUSARTx->DR |= dummy_write;

    (void)dummy_read;
    (void)dummy_write;
}

uint8_t usart_send_data(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuf, uint32_t txLen)
{
    uint8_t txState = pUSART_Handle->TX_STATE;
    // 1. Ensure that transmission is not already happening
    if (txState != USART_BUSY_IN_TX)
    {
        pUSART_Handle->TX_STATE = USART_BUSY_IN_TX;
        pUSART_Handle->TxBuf = pTxBuf;
        pUSART_Handle->TxLen = txLen;

        usart_tx_interrupt_enable(pUSART_Handle->USARTx);
        usart_tc_interrupt_enable(pUSART_Handle->USARTx);
    }
    return txState;
}

void usart_irq_handler(USART_Handle_t *pUSART_Handle)
{
    uint8_t interrupt_trigger, interrupt_enabled;
    /************************ USART TXE IT ***********************/
    interrupt_trigger = pUSART_Handle->USARTx->SR & (USART_SR_TXE_Msk);
    interrupt_enabled = pUSART_Handle->USARTx->CR1 & (USART_CR1_TXEIE_Msk);
    if (interrupt_trigger && interrupt_enabled)
    {
        if (pUSART_Handle->TX_STATE == USART_BUSY_IN_TX)
        {
            if (pUSART_Handle->TxLen > 0)
            {
                // transmit 1 data word of len 8 with one stop bit and no parity control
                pUSART_Handle->USARTx->DR = (*(pUSART_Handle->TxBuf) & ((uint8_t) 0xFF));
                pUSART_Handle->TxBuf++;
                pUSART_Handle->TxLen--;
            }
            if (pUSART_Handle->TxLen < 1)
            {
                // DISABLE TXE INTERRUPTS TRANSMISSION IS COMPLETE
                usart_tx_interrupt_disable(pUSART_Handle->USARTx);
            }
        }
    }
    /************************ USART TC IT  ***********************/
    interrupt_trigger = pUSART_Handle->USARTx->SR & (USART_SR_TC_Msk);
    interrupt_enabled = pUSART_Handle->USARTx->CR1 & (USART_CR1_TCIE_Msk);

    if (interrupt_trigger && interrupt_enabled)
    {
        if(pUSART_Handle->TX_STATE == USART_BUSY_IN_TX)
        {
            if (pUSART_Handle->TxLen < 1)
            {
                // usart_clear_tc(pUSART_Handle->USARTx);
                pUSART_Handle->USARTx->SR &= ~(USART_SR_TC);
                pUSART_Handle->USARTx->CR1 &= ~(USART_CR1_TCIE);

                pUSART_Handle->TX_STATE = USART_TX_READY;
                pUSART_Handle->TxBuf = NULL;
                pUSART_Handle->TxLen = 0;
            }
        }
    }
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

void _putchar(char c)
{
    // 8-bit data transmission with 1 stop bit and no parity bit enabled
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
