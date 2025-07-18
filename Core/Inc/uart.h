#ifndef _UART_H
#define _UART_H

#include "main.h"

#define CLK_HZ      16000000UL

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

/*********************************************
 *          UART User-defined Structs
 *********************************************/
typedef struct 
{
    uint8_t     USART_Mode;
    uint8_t     USART_ParityCtrl;
    uint8_t     USART_NO_OF_STOP_BITS;
    uint8_t     USART_WordLen;
    uint32_t    USART_BaudRate;
    uint8_t     USART_HWFlowCtrl;
}USART_Config_t;

 typedef struct
{
    uint8_t     TX_STATE;
    uint8_t     *TxBuf;
    uint32_t    TxLen;
    USART_Config_t USART_Confg;
    USART_TypeDef *USARTx;
}USART_Handle_t;

/*********************************************
 *          USART User-defined Enums
 *********************************************/
typedef enum
{
    USART_WORD_LEN8,
    USART_WORD_LEN9
} USART_WordLen;

typedef enum
{
    USART_STOP_BITS1,
    USART_STOP_BITS0_5,
    USART_STOP_BITS2,
    USART_STOP_BITS1_5
}USART_StopBitCount;

typedef enum
{
    USART_RTS_CTS,
    USART_RTS_ONLY,
    USART_CTS_ONLY,
    USART_NO_HWFlowCtrl
}USART_HWFlowCtrlOp;

typedef enum
{
    USART_PARITY_CRL_EVEN_EN,
    USART_PARITY_CRL_ODD_EN,
    USART_PARITY_CRL_DI
} USART_ParityCrl;

typedef enum
{
    USART_TX_READY,
    USART_BUSY_IN_TX
} USART_TxStates;

typedef enum
{
	USART1_IRQ_NO = 37,
    USART2_IRQ_NO,
    USART3_IRQ_NO,
    UART4_IRQ_NO = 52,
    UART5_IRQ_NO,
    USART6_IRQ_NO = 71
}USART_IRQn;

/*********************************************
 *          USART User-defined APIs
 *********************************************/

void usart_init(USART_Handle_t *USART_Handle);
void usart_enable_peripheral(USART_TypeDef *pUSARTx);
void usart_configure(USART_Handle_t *USART_Handle);
void _putchar(char c);
/* TODO: ABSTRACT INTERRUPT ENABLING FUNCTION FROM IO FILE TO INTERRUPT HEADER */

uint8_t usart_send_data(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuf, uint32_t txLen);
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void usart_irq_handler(USART_Handle_t *pUSART_Handle);

void USART_IRQEnableInterrupt(uint16_t EXTI_IRQn);
void USART_IRQDisableIT(uint16_t EXTI_IRQn);

#endif