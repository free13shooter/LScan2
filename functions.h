//--------------------------------------------------------------
// File     : functions.h
//--------------------------------------------------------------



#ifndef __FUNCTIONS_H
#define __FUNCTIONS_H

typedef struct dma_config{
  DMA_Stream_TypeDef * DMAx_Streamy;
  uint32_t DMA_Channel_x;
  
  uint32 DMA_PeripheralBaseAddr ;
  uint32 DMA_PeripheralDataSize ;
  uint32 DMA_MemoryDataSize ;
  uint32 DMA_PeripheralInc ;
  uint32 DMA_BufferSize ;
  uint32 DMA_Mode;
  uint32 DMA_Priority;
  uint32 DMA_FIFOMode;
  uint32 DMA_FIFOThreshold;
  uint32 DMA_MemoryBurst;
  uint32 DMA_PeripheralBurst;
  
  uint32 DMA_FLAG_FEIF;
  uint32 DMA_FLAG_DMEIF;
  uint32 DMA_FLAG_TEIF;
  uint32 DMA_FLAG_HTIF;
  uint32 DMA_FLAG_TCIF;
   
  IRQn_Type  DMAx_Streamy_IRQn;
  void (*DMAx_Streamy_IRQHandler)(void); 
}

void DMA_RX_TX_config(dma_config* dma);
//--------------------------------------------------------------
#endif // __FUNCTIONS_H