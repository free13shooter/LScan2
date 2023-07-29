#include "functions.h"
//________________________________________________________________________________________________________//
static volatile uint8_t DMAEndOfTransferRx=0;
static void DMA_transfer( dma_config* dma,
	BOOL RX,		          // FALSE,if TX              
	const BYTE *buff,	  		           
	UINT buf_size 			
)
{
	 if((uint32_t)buff<SRAM1_BASE)
   {
     return ;
     //while (1);
     //ОШИБКА-память должна быть не CORE-COUPED-MEMORY !
     //DMA не работает с CCM-RAM !!!
   }
   
   DMA_InitTypeDef DMA_InitStructure;
   DMA_StructInit(&DMA_InitStructure);
   
   /* shared DMA configuration values */
   DMA_InitStructure.DMA_Channel = dma.DMA_Channel_x;
   DMA_InitStructure.DMA_PeripheralBaseAddr = dma.DMA_PeripheralBaseAddr;
   DMA_InitStructure.DMA_PeripheralDataSize = dma.DMA_PeripheralDataSize;
   DMA_InitStructure.DMA_MemoryDataSize = dma.DMA_MemoryDataSize;
   DMA_InitStructure.DMA_PeripheralInc = dma.DMA_PeripheralInc;
   DMA_InitStructure.DMA_BufferSize = buf_size;
   DMA_InitStructure.DMA_Mode =     dma.DMA_Mode;
   DMA_InitStructure.DMA_Priority = dma.DMA_Priority;
   DMA_InitStructure.DMA_FIFOMode = dma.DMA_FIFOMode;
   DMA_InitStructure.DMA_FIFOThreshold = dma.DMA_FIFOThreshold;
   DMA_InitStructure.DMA_MemoryBurst = dma.DMA_MemoryBurst;
   DMA_InitStructure.DMA_PeripheralBurst = dma.DMA_PeripheralBurst;
   
  
   DMA_Cmd(DMA_STREAM_SPI_SD_RX, DISABLE);
   DMA_Cmd(DMA_STREAM_SPI_SD_TX, DISABLE);   
   
   SPI_I2S_DMACmd(SPI_SD,SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
   
   DMA_ClearFlag(DMA_STREAM_SPI_SD_RX,DMA_FLAG_SPI_SD_TC_RX|DMA_FLAG_SPI_SD_FE_RX|DMA_FLAG_SPI_SD_TE_RX|
                  DMA_FLAG_SPI_SD_HT_RX|DMA_FLAG_SPI_SD_DE_RX);
   DMA_ClearFlag(DMA_STREAM_SPI_SD_TX,DMA_FLAG_SPI_SD_TC_TX|DMA_FLAG_SPI_SD_FE_TX|DMA_FLAG_SPI_SD_TE_TX|
                  DMA_FLAG_SPI_SD_HT_TX|DMA_FLAG_SPI_SD_DE_TX);
   
   if ( RX ) 
   {      
      DMA_DeInit(DMA_STREAM_SPI_SD_RX);
      /* DMA1 configuration RX ---------------------------------------------*/
      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buff;
      DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
      DMA_Init(DMA_STREAM_SPI_SD_RX, &DMA_InitStructure);
      
      // DMA1 c  configuration TX ---------------------------------------------
      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rw_workbyte;
      DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
      DMA_Init(DMA_STREAM_SPI_SD_TX, &DMA_InitStructure);
   } 
   else
   {   
      DMA_DeInit(DMA_STREAM_SPI_SD_TX);
      // DMA1 configuration RX ---------------------------------------------
      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rw_workbyte;
      DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
      DMA_Init(DMA_STREAM_SPI_SD_RX, &DMA_InitStructure);
      /* DMA1 configuration TX ---------------------------------------------*/
      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buff;
      DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
      DMA_Init(DMA_STREAM_SPI_SD_TX, &DMA_InitStructure);

   }
   
   DMA_Cmd(DMA_STREAM_SPI_SD_RX, ENABLE);
   DMA_Cmd(DMA_STREAM_SPI_SD_TX, ENABLE);
   SPI_I2S_DMACmd(SPI_SD, SPI_I2S_DMAReq_Rx|SPI_I2S_DMAReq_Tx, ENABLE);
   while (DMA_GetFlagStatus(DMA_STREAM_SPI_SD_RX,DMA_FLAG_SPI_SD_TC_RX) == RESET);
   while (DMA_GetFlagStatus(DMA_STREAM_SPI_SD_TX,DMA_FLAG_SPI_SD_TC_TX) == RESET);
   DMA_Cmd(DMA_STREAM_SPI_SD_RX, DISABLE);
   DMA_Cmd(DMA_STREAM_SPI_SD_TX, DISABLE);
   /* Disable SPI1 RX/TX request */
   SPI_I2S_DMACmd(SPI_SD,SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
}
//________________________________________________________________________________________________________//
void SPI1_transmit(uint8_t* txbuf,uint32_t txlen,uint8_t* rxbuf,uint32_t rxlen){
  SPI1_tx(txbuf,txlen);SPI1_rx( rxbuf,rxlen);
}

void SPI1_tx(uint8_t* txbuf,uint32_t txlen){
  
  if(!(GPIOA->IDR & GPIO_Pin_4))SPI1->CR1 &= SPI_NSSInternalSoft_Reset;
  
  if(txbuf && txlen){
    while(SPI_tx_len/* && (SPI1->CR2 & SPI_CR2_TXEIE)*/){
      if(GPIOA->IDR & GPIO_Pin_4){
        SPI1->CR2 &=(uint16_t)~(SPI_CR2_RXNEIE|SPI_CR2_TXEIE);
        rxLen=txLen=0;
        SPI1->CR1 |= SPI_NSSInternalSoft_Set;
        return;
      }
      
      if((SPI1->CR2 & SPI_CR2_RXNEIE)==0)SPI1->CR2 |= SPI_CR2_TXEIE;
    }
    

    SPI_tx_buf=txbuf;
    SPI_tx_len=txlen;
    
    if((SPI1->CR2 & SPI_CR2_TXEIE)==0){
      SPI1->CR2 |= SPI_CR2_TXEIE;
    }
  }
} 

void SPI1_rx(uint8_t* rxbuf,uint32_t rxlen){
  
  if(!(GPIOA->IDR & GPIO_Pin_4))SPI1->CR1 &= SPI_NSSInternalSoft_Reset;
  
  if(rxbuf && rxlen){
    while(SPI_rx_len/* && (SPI1->CR2 & SPI_CR2_RXNEIE)*/){
      if(0){//GPIOA->IDR & GPIO_Pin_15){
        SPI1->CR2 &=(uint16_t)~(SPI_CR2_RXNEIE|SPI_CR2_TXEIE);
        rxLen=txLen=0;
        SPI1->CR1 |= SPI_NSSInternalSoft_Set;
        memset(rxbuf,0,rxlen);
        return;
      }
      
      //if((SPI1->CR2 & SPI_CR2_RXNEIE)==0)
        SPI1->CR2 |= SPI_CR2_RXNEIE;
    };
    
    
    SPI_rx_buf=rxbuf;
    SPI_rx_len=rxlen;
    //if((SPI1->CR2 & SPI_CR2_RXNEIE)==0){
      SPI1->CR2 |= SPI_CR2_RXNEIE;
    //}
  }
}
//________________________________________________________________________________________________________//