#include "UART.h"
#include "DLib_Product_string.h" //memcpy

//------------------------------------------------------------------------------
extern uint16_t IsocOutPacketSize;
//extern uint8_t* IsocOutWrPtr;
//extern uint8_t* IsocOutRdPtr;
extern uint8_t* iso_buff_over;//первый байт за буфером
//extern uint32_t LS_BUF_SIZE;
extern uint8_t USB_IsoRx_Buffer [];
extern uint8_t LScanStatus;
//------------------------------------------------------------------------------
#define U1_IRQn             USART1_IRQn
#define U1_IRQHandler       USART1_IRQHandler
#define U1RX_Stream         DMA2_Stream5
#define U1RX_DMA_IRQn       DMA2_Stream5_IRQn
#define U1RX_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define U1RX_DMA_FLAG_HTIF  DMA_IT_HTIF5  //  1/2 filled
#define U1RX_DMA_FLAG_TCIF  DMA_IT_TCIF5  //  transfer complete (full FIFO)
#define U1RX_DMA_FLAG_FEIF  DMA_IT_FEIF5  //  FIFO error
#define U1RX_DMA_FLAG_TEIF  DMA_IT_TEIF5  //  transfer error

#define U1TX_Stream DMA2_Stream7
#define U1TX_DMA_IRQn       DMA2_Stream7_IRQn
#define U1TX_DMA_IRQHandler DMA2_Stream7_IRQHandler
#define U1TX_DMA_FLAG_HTIF  DMA_IT_HTIF7  //  1/2 filled
#define U1TX_DMA_FLAG_TCIF  DMA_IT_TCIF7  //  transfer complete (full FIFO)
#define U1TX_DMA_FLAG_FEIF  DMA_IT_FEIF7  //  FIFO error
#define U1TX_DMA_FLAG_TEIF  DMA_IT_TEIF7  //  transfer error
//------------------------------------------------------------------------------
#pragma data_alignment = 4 
ICP U1rx[2*sizeof(ICP)];
#pragma data_alignment = 4 
uint8_t U1tx[UART_LEN];

volatile int u1rxcnt=0; //dbg

static void process_cmd(ICMD cmd);

static void StartU1RXstream();
static void StopU1RXstream();
//------------------------------------------------------------------------------
void UART1_init()
{
  //DMA 16 bytes FIFO per stream
  //--------------------USART 1--------------------------
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//84 MHz
  /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
  GPIO_InitTypeDef gpio;
  GPIO_StructInit(&gpio);
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//TX|RX
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio);
    
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
  
  //----------USART init------------
  USART_InitTypeDef usart;
  USART_StructInit(&usart);
  usart.USART_BaudRate = (uint32_t)UART1_BR;
    
  USART_ClearITPendingBit(USART1, USART_IT_TXE);
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  USART_OverSampling8Cmd(USART1,ENABLE); //увеличить скорость путем уменьшения семплирования
  //USART_OneBitMethodCmd(USART2,ENABLE); //уменьшить количество стробов
  
  USART_Init(USART1, &usart);	
 
  //------Peripheral DMA init USART1 RX=DMA2_stream5 Channel4---------------
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  //------Peripheral DMA init USART1 TX=DMA2_stream7 Channel4---------------
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U1tx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize =(uint32_t)UART_LEN;//NDTR in data unit
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(U1TX_Stream, &DMA_InitStructure);
  //-------------IT---------------------
  DMA_ClearITPendingBit(U1TX_Stream,U1TX_DMA_FLAG_HTIF|U1TX_DMA_FLAG_TCIF|U1TX_DMA_FLAG_FEIF|U1TX_DMA_FLAG_TEIF);
  USART_DMACmd(USART1, USART_DMAReq_Tx,ENABLE);
  DMA_ITConfig(U1TX_Stream, DMA_IT_TE|DMA_IT_TC, ENABLE);//прерывание по ошибке 
  
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = U1_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = U1RX_DMA_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = U1TX_DMA_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  //---------------------------------------------
  USART_Cmd(USART1, ENABLE);
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
}
//-------------UART1 DMA rx IRQ-------------------------------------------------
void U1RX_DMA_IRQHandler(void)
{
  uint8_t error=0;
  //-------check errors-----------
  USART_GetITStatus(USART1, USART_IT_IDLE);//очистка флага
    
  if (DMA_GetITStatus(U1RX_Stream,U1RX_DMA_FLAG_TEIF) != RESET)//transfer error
  {
    DMA_ClearITPendingBit(U1RX_Stream,U1RX_DMA_FLAG_TEIF);
    error=1;
  }
  if(DMA_GetITStatus(U1RX_Stream,U1RX_DMA_FLAG_FEIF) != RESET)//FIFO error
  {
    DMA_ClearITPendingBit(U1RX_Stream,U1RX_DMA_FLAG_FEIF);
    error=1;
  }
  
  if(error)
  {
    ORANGE_LED_ON();//O
    return;
  }
  //-------no errors--------------
  BLUE_LED_ON();//B
  ICP indata;
  //uint32_t cnt=U1RX_Stream->NDTR;
  if(DMA_GetITStatus(U1RX_Stream,U1RX_DMA_FLAG_HTIF) != RESET)
  {
    DMA_ClearITPendingBit(U1RX_Stream,U1RX_DMA_FLAG_HTIF);//1/2
    indata=U1rx[0];
  }
  
  if(DMA_GetITStatus(U1RX_Stream,U1RX_DMA_FLAG_TCIF) != RESET)//full
  {
    DMA_ClearITPendingBit(U1RX_Stream,U1RX_DMA_FLAG_TCIF);
    indata=U1rx[1];
  }
    
  u1rxcnt+=8;
  //-------------проверка команды-------------------
  if((ICMD)indata.cmd>=(ICMD)IC_GET_STREAM_PAK_SIZE)//принята команда
  {
    process_cmd((ICMD)indata.cmd);
    //структура координат/цветов в любом случае целая,копируем в буфер
  }
  //-----------------------------------------------------
  //NDTR - This register can be written only when the stream is disabled
  memcpy((void*)IsocOutWrPtr,(uint8_t*)&indata,sizeof(ICP));//поместить в потоковый буфер
  //обновить указатель
  if(IsocOutWrPtr+sizeof(ICP)>=iso_buff_over)IsocOutWrPtr+=sizeof(ICP)-LS_BUF_SIZE;//circular offset
  else IsocOutWrPtr+=sizeof(ICP);
}
//-------------UART1 DMA tx IRQ-------------------------------------------------
void U1TX_DMA_IRQHandler(void)
{
  //-------check errors-----------
  if(DMA_GetITStatus(U1TX_Stream, U1TX_DMA_FLAG_TEIF) != RESET)//trans error
  {
    DMA_ClearITPendingBit(U1TX_Stream, U1TX_DMA_FLAG_TEIF);
    ORANGE_LED_ON();//O
    //while(1); //DBG
  }
  
  if(DMA_GetITStatus(U1TX_Stream, U1TX_DMA_FLAG_TCIF) != RESET)
  {
    DMA_ClearITPendingBit(U1TX_Stream, U1TX_DMA_FLAG_TCIF);
    GREEN_LED_ON();//G
  }  
  
  UDMA_stop(U1TX_Stream);
}
//----------------command over UART1 processing---------------------------------
void U1_IRQHandler(void)
{
  static uint8_t rxcnt=0;
  static ICMD cmd=IC_NO;
  //-----------------запрос RX,перейти к обработке команды-----------------------
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    GREEN_LED_ON();//G
    rxcnt++;
    uint8_t v=((uint8_t)USART_ReceiveData(USART1) & (uint8_t)0xFF);
    if(rxcnt==2)
    {
      cmd=(ICMD)v;
    }
    
    if(rxcnt==8)
    {
      process_cmd(cmd);
      cmd=IC_NO;
      rxcnt=0;
    }
  }//RX start 
  //----------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
//обработка входящей команды
static void process_cmd(ICMD cmd) 
{
  uint8_t txlen=0;
  int dis=0;
  //----------select command------------
  switch(cmd)
  {
   case IC_GET_LVER:
      U1tx[0]=(uint8_t)2;//2=LScan2 version
      txlen=1;
      break;
   case IC_SET_USB_DISABLE:
      //Disconnect_FS_USB_Device(&USB_OTG_dev);//usb core device;
      break;
   case IC_GET_STREAM_PAK_SIZE://isoc packet size
      U1tx[0]=(uint8_t)(IsocOutPacketSize/8);
      txlen=1;
      break;
   case IC_GET_STREAM_BUF_SIZE://buff size in packets
      U1tx[0]=(uint8_t)((int)LS_BUF_SIZE/(int)IsocOutPacketSize);
      txlen=1;
      break;
   case IC_GET_BUF_FREE_8://free buff size in bytes/8
      dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;
      *((uint16_t*)U1tx)=(uint16_t)(dis/8);
      txlen=2;
      break;
   case IC_GET_BUF_FREE_PACKS://free in buff (in packets)
      dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;
      U1tx[0]=(uint8_t)(dis/(int)IsocOutPacketSize);  
      txlen=1;
      break;
   case IC_GET_BUF_FREE_256://free in buff(256-bytes parts)
      dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;//буфер свободен
      U1tx[0]=(uint8_t)(dis/256); //частей 
      txlen=1;
      break;
   //-------------------  
   case IC_NO:
   default:
      dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;
      U1tx[0]=(uint8_t)(dis/(int)IsocOutPacketSize);  
      txlen=1;
      break;
   }//switch(cmd)
  
  if(txlen==0)return;
  //if(USART_GetFlagStatus(USART1,USART_FLAG_TXE)!=RESET)USART1->DR =((uint16_t)U1tx[0])&(uint16_t)0x00FF;
  UDMA_transfer(U1TX_Stream,txlen,(uint32_t)U1tx);
  
  txlen=0;
}
//------------------------------------------------------------------------------
static void StartU1RXstream()
{
  //отключить отслеживание команды активации потока
  USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
  
  UDMA_stop(U1RX_Stream);
  //------Peripheral DMA init USART1 RX=DMA2_stream5 Channel4---------------
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U1rx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize =(uint32_t)(2*sizeof(ICP));//NDTR in data unit
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//FIFO->buf->memcpy
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;//8 bytes
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(U1RX_Stream, &DMA_InitStructure);
  //-------------IT---------------------
  DMA_ClearITPendingBit(U1RX_Stream,U1RX_DMA_FLAG_HTIF|U1RX_DMA_FLAG_TCIF|U1RX_DMA_FLAG_FEIF|U1RX_DMA_FLAG_TEIF);
  
  USART_DMACmd(USART1, USART_DMAReq_Rx,ENABLE);
  DMA_ITConfig(U1RX_Stream, DMA_IT_HT, ENABLE);//прерывание по 1/2 FIFO
  DMA_ITConfig(U1RX_Stream, DMA_IT_TC, ENABLE);//прерывание по заполнению
  DMA_ITConfig(U1RX_Stream, DMA_IT_TE, ENABLE);//прерывание по ошибке 
  DMA_ITConfig(U1RX_Stream, DMA_IT_FE, ENABLE);//прерывание по ошибке FIFO
  
  //---------------------------------------------
  DMA_Cmd(U1RX_Stream,ENABLE);//start rx
}
//------------------------------------------------------------------------------
static void StopU1RXstream()
{
  DMA_Cmd(U1RX_Stream,DISABLE);//rx
  while (U1RX_Stream->CR & (uint32_t)DMA_SxCR_EN);
  DMA_ITConfig(U1RX_Stream, DMA_IT_TC, DISABLE);//прерывание по заполнению
  DMA_ITConfig(U1RX_Stream, DMA_IT_TE, DISABLE);//прерывание по ошибке 
  DMA_ITConfig(U1RX_Stream, DMA_IT_FE, DISABLE);//прерывание по ошибке FIFO
  DMA_ITConfig(U1RX_Stream, DMA_IT_HT, DISABLE);//прерывание по 1/2 буфера FIFO 
  USART_DMACmd(USART1, USART_DMAReq_Rx,DISABLE);
    
  DMA_ClearITPendingBit(U1RX_Stream,U1RX_DMA_FLAG_TEIF);
  DMA_ClearITPendingBit(U1RX_Stream,U1RX_DMA_FLAG_FEIF);
  
  //USART_ITConfig(USART1,USART_IT_IDLE,DISABLE);//отключить отслеживание 
  //включить отслеживание данных(команды активации потока)
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
}
//------------------------------------------------------------------------------