#include "dma.h"
#include "interrupts.h"

#define NUMBER_OF_ITEMS_TO_TX   (4U)

static void dma_clear_status_regs(void)
{
    DMA2->HIFCR = (uint32_t) 0xFFFF;
    DMA2->LIFCR = (uint32_t) 0xFFFF;
}

static void dma_configure_transfer(void)
{
    DMA2_Stream0->CR &= ~(DMA_SxCR_DIR);
    DMA2_Stream0->CR |= (DMA_PERI_SIZE_HalfWord << DMA_SxCR_PSIZE_Pos) | (DMA_PERI_SIZE_HalfWord << DMA_SxCR_MSIZE_Pos);
    DMA2_Stream0->CR |= (ENABLE << DMA_SxCR_CIRC_Pos);
}

static void dma_transfer_complete_interrupt_enable(void)
{
    IT_SetInterruptPriority(DMA2_STREAM0_IRQ_NO, 1);
    IT_IRQEnableInterrupt(DMA2_STREAM0_IRQ_NO);
    DMA2_Stream0->CR |= (DMA_SxCR_TCIE);
}

void dma_configure(uint32_t src, uint32_t dst)
{
    // set src and dst for data to be transferred to/from respectively
    DMA2_Stream0->PAR = src;
    DMA2_Stream0->M0AR = dst;
    DMA2_Stream0->CR |= (DMA_SxCR_MINC);

    // Configure number of data items to be tx
    DMA2_Stream0->NDTR = (uint16_t) NUMBER_OF_ITEMS_TO_TX;
    DMA2_Stream0->CR &= ~(DMA_CH_0 << DMA_SxCR_CHSEL_Pos);

    // Set DMA as flow controller
    DMA2_Stream0->CR &= ~(DMA_SxCR_PFCTRL);

    dma_configure_transfer();
    dma_transfer_complete_interrupt_enable();
}

void dma_init(uint32_t src, uint32_t dst)
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN);
    DMA2_Stream0->CR &= ~(DMA_SxCR_EN);
    while (DMA2_Stream0->CR & (DMA_SxCR_EN_Msk));
    dma_clear_status_regs();
    dma_configure(src, dst);
    DMA2_Stream0->CR |= (DMA_SxCR_EN);
}
