#ifndef _DMA_H
#define _DMA_H

#include "main.h"

#define DMA2_STREAM0_IRQ_NO     (56U)
/**************************
 *          DMA enums
 **************************/
typedef enum
{
    DMA_CH_0 = 7,
    DMA_CH_1 = 1,
    DMA_CH_2,
    DMA_CH_3,
    DMA_CH_4,
    DMA_CH_5,
    DMA_CH_6,
    DMA_CH_7
} dma_channel_e;

typedef enum
{
    DMA_PERI_SIZE_BYTE,
    DMA_PERI_SIZE_HalfWord,
    DMA_PERI_SIZE_Word
} dma_peri_size_e;

/**************************
 *          DMA APIs
 **************************/
void dma_init(uint32_t src, uint32_t dst);
void dma_configure(uint32_t src, uint32_t dst);
// void dma_start_transfer(void);

#endif