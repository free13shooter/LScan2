#include "UART.h"
#include "DLib_Product_string.h" //memcpy
#include "core.h" //timers
//------------------------------------------------------------------------------
extern USB_OTG_CORE_HANDLE  USB_OTG_dev;//usb core device
extern uint16_t IsocOutPacketSize;
extern uint8_t* iso_buff_over;//первый байт за буфером
extern uint8_t USB_IsoRx_Buffer [];
extern uint8_t LScanStatus;
extern uint8_t* LS_BUF_END;
//------------------------------------------------------------------------------
#define U2_IRQn             USART2_IRQn
#define U2_IRQHandler       USART2_IRQHandler
#define U2RX_Stream         DMA1_Stream5
#define U2RX_DMA_IRQn       DMA1_Stream5_IRQn
#define U2RX_DMA_IRQHandler DMA1_Stream5_IRQHandler

#define U2RX_DMA_FLAG_HTIF  DMA_FLAG_HTIF6  //  1/2 filled
#define U2RX_DMA_FLAG_TCIF  DMA_FLAG_TCIF6  //  transfer complete (full FIFO)
#define U2RX_DMA_FLAG_FEIF  DMA_FLAG_FEIF6  //  FIFO error
#define U2RX_DMA_FLAG_TEIF  DMA_FLAG_TEIF6  //  transfer error
#define U2RX_DMA_IT_HTIF    DMA_IT_HTIF5    //  1/2 filled
#define U2RX_DMA_IT_TCIF    DMA_IT_TCIF5    //  transfer complete (full FIFO)
#define U2RX_DMA_IT_FEIF    DMA_IT_FEIF5    //  FIFO error
#define U2RX_DMA_IT_TEIF    DMA_IT_TEIF5    //  transfer error

#define U2TX_Stream         DMA1_Stream6
#define U2TX_DMA_IRQn       DMA1_Stream6_IRQn
#define U2TX_DMA_IRQHandler DMA1_Stream6_IRQHandler
#define U2TX_DMA_FLAG_HTIF  DMA_FLAG_HTIF6  //  1/2 filled
#define U2TX_DMA_FLAG_TCIF  DMA_FLAG_TCIF6  //  transfer complete (full FIFO)
#define U2TX_DMA_FLAG_FEIF  DMA_FLAG_FEIF6  //  FIFO error
#define U2TX_DMA_FLAG_TEIF  DMA_FLAG_TEIF6  //  transfer error
#define U2TX_DMA_IT_HTIF    DMA_IT_HTIF6    //  1/2 filled
#define U2TX_DMA_IT_TCIF    DMA_IT_TCIF6    //  transfer complete (full FIFO)
#define U2TX_DMA_IT_FEIF    DMA_IT_FEIF6    //  FIFO error
#define U2TX_DMA_IT_TEIF    DMA_IT_TEIF6    //  transfer error
//------------------------------------------------------------------------------
#pragma data_alignment = 4 
ICP U2rx[2];
#pragma data_alignment = 4 
uint8_t   U2tx[UART_LEN];
uint8_t   p2tx=0;
int u2txleft=0;

volatile int u2rxcnt=0; //dbg



extern IMODE ICmode;//тип соединени€
SystemTimer* U2DMA_timer=NULL;//таймер наблюдени€ за DMA
//-------наблюдение за бездействием DMA RX------------
static volatile uint32_t idleDMAcycles=0;//счетчик циклов бездействи€ RX

static void process_cmd(ICMD cmd);//обработчик команд
static void StartU2RXstream();//перезагрузка DMA RX

//------------------------------------------------------------------------------
void UART2_init()
{
  //DMA 16 bytes FIFO per stream
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//42 MHz
 
  //------------USART2----------
  GPIO_InitTypeDef gpio;
  GPIO_StructInit(&gpio);
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_Pin = GPIO_Pin_2;//TX
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);
  
  gpio.GPIO_Pin = GPIO_Pin_6;//RX
  GPIO_Init(GPIOD, &gpio);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
  //----------USART init------------
  USART_InitTypeDef usart;
  USART_StructInit(&usart);
  usart.USART_BaudRate = (uint32_t)UART2_BR;
  
  USART_ClearITPendingBit(USART2, USART_IT_TXE);
  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  USART_OverSampling8Cmd(USART2,ENABLE); //увеличить скорость путем уменьшени€ семплировани€
  USART_Init(USART2, &usart);	
 //------Peripheral DMA init Channel4---------------
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  //------Peripheral DMA init USART2 TX=DMA2_stream6 Channel4---------------
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART2->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U2tx;
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
  DMA_Init(U2RX_Stream, &DMA_InitStructure);
  //-------------IT---------------------
  DMA_ClearFlag(U2TX_Stream,U2TX_DMA_FLAG_HTIF|U2TX_DMA_FLAG_TCIF|U2TX_DMA_FLAG_FEIF|U2TX_DMA_FLAG_TEIF);
  //USART_DMACmd(USART2, USART_DMAReq_Tx,ENABLE);
  //DMA_ITConfig(U2TX_Stream, DMA_IT_TE|DMA_IT_TC, ENABLE);//прерывание по ошибке 
  
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = U2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = (uint8_t)WLAN_UART2_IT_PRIO;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = (uint8_t)0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;//TX !
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = U2RX_DMA_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = (uint8_t)WLAN_UART2_DMA_RX_IT_PRIO;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = (uint8_t)0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = U2TX_DMA_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = (uint8_t)WLAN_UART2_DMA_TX_IT_PRIO;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = (uint8_t)0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  //---------------------------------------------
  USART_Cmd(USART2, ENABLE);
  //стартуем DMA rx.ѕрерывание USART_IT_RXNE не требуетс€.
  StartU2RXstream();
  //отслеживание зависани€ DMA rx на врем€ TRX_IDLE_SEC
  U2DMA_timer=SetPeriodicSystemTimer((uint16_t)1000,&U2timerProc);
  //-------------------------U2DMA_timer= AddSystemTask(&U2timerProc,NULL);
}
//-------------UART DMA rx IRQ-------------------------------------------------
void U2RX_DMA_IRQHandler(void)
{
  uint8_t error=0;
  //-------check errors-----------
  if (DMA_GetITStatus(U2RX_Stream,U2RX_DMA_IT_TEIF) != RESET)//transfer error
  {
    DMA_ClearITPendingBit(U2RX_Stream,U2RX_DMA_IT_TEIF);
    error=1;
  }
  /*if(DMA_GetITStatus(U2RX_Stream,U2RX_DMA_IT_FEIF) != RESET)//FIFO error
  {
    DMA_ClearITPendingBit(U2RX_Stream,U2RX_DMA_IT_FEIF);
    error=1;
  }*/
  
  if(error)
  {
    ORANGE_LED_ON();//O
    return;
  }
  //-------no errors--------------
  //BLUE_LED_ON();//B
  ICP indata={0};
  uint8_t reload=1;//произошла перезагрузка
  
  if(DMA_GetITStatus(U2RX_Stream,U2RX_DMA_IT_TCIF) != RESET)//full
  {
    DMA_ClearITPendingBit(U2RX_Stream,U2RX_DMA_IT_TCIF);
    indata=U2rx[1];
    reload=0;
  }
  else if(DMA_GetITStatus(U2RX_Stream,U2RX_DMA_IT_HTIF) != RESET)
  {
    DMA_ClearITPendingBit(U2RX_Stream,U2RX_DMA_IT_HTIF);//1/2
    indata=U2rx[0];
    reload=0;
  }
  
  idleDMAcycles=0;
  if(reload)return;//обработка не требуетс€
    
  if(indata.discrMs && ICmode==IM_WLAN)//приоритет воспроизведени€ с карты и OTG
  {
    /*if(LScanStatus==STATUS_PLAY && IsocOutWrPtr+sizeof(ICP)==IsocOutRdPtr)
    {
     //критическа€ ошибка,опережение записи.—емпл придетс€ пропустить.
      IsocOutRdPtr=IsocOutWrPtr=USB_IsoRx_Buffer;
      Beep(4000,90);//DBG
    }
    else
    {*/
      /*if(indata.lx|indata.ly|indata.hxhy ==0)
      {
        Beep(4000,90);//DBG
      }*/
      
      *(ICP*)IsocOutWrPtr=*(&indata);
          //memcpy(IsocOutWrPtr,(uint8_t*)&indata,sizeof(ICP));//поместить в потоковый буфер
      //обновить указатель
      if(IsocOutWrPtr+sizeof(ICP)>LS_BUF_END)IsocOutWrPtr=USB_IsoRx_Buffer;
      else IsocOutWrPtr+=sizeof(ICP);
      u2rxcnt++;
    //}
  }
  else if(ICmode<=IM_USB)
  {
    if(ICmode==IM_USB)Disconnect_FS_USB_Device(&USB_OTG_dev);//usb core device;
    ICmode=IM_WLAN;
  }
  
  /*
  
  if(indata.cmd==0)indata.cmd=IC_GET_BUF_FREE_256;//отправить синхро
  */
 
  //-------------проверка команды-------------------
  if(indata.cmd)//прин€та команда
  {
    if(indata.cmd==IC_GET_BUF_FREE_256 && USART_GetFlagStatus(USART2,USART_FLAG_TXE)!=RESET)
    {
      int dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;//буфер свободен
      U2tx[0]=(uint8_t)(dis/256); //частей 
      USART2->DR =(uint16_t)(U2tx[0])&(uint16_t)0x00FF;//free in buff(256-bytes parts)
      return;
    }  
    else process_cmd((ICMD)indata.cmd);
  }
  
  //-----------------------------------------------------
  //NDTR - This register can be written only when the stream is disabled
}
//-------------UART DMA tx IRQ-------------------------------------------------
void U2TX_DMA_IRQHandler(void)
{
  idleDMAcycles=0;
  //-------check errors-----------
  if(DMA_GetITStatus(U2TX_Stream, U2TX_DMA_IT_TEIF) != RESET)//trans error
  {
    DMA_ClearITPendingBit(U2TX_Stream, U2TX_DMA_IT_TEIF);
    ORANGE_LED_ON();//O
    //while(1); //DBG
  }
  
  if(DMA_GetITStatus(U2TX_Stream, U2TX_DMA_IT_TCIF) != RESET)
  {
    DMA_ClearITPendingBit(U2TX_Stream, U2TX_DMA_IT_TCIF);
    GREEN_LED_ON();//G
  }
  
  UDMA_stop(U2TX_Stream);
  USART_DMACmd(USART2, USART_DMAReq_Tx,DISABLE);
  DMA_ClearFlag(U2TX_Stream,U2TX_DMA_FLAG_HTIF|U2TX_DMA_FLAG_TCIF|U2TX_DMA_FLAG_FEIF|U2TX_DMA_FLAG_TEIF);
}
//----------------command over UART1 processing---------------------------------
void U2_IRQHandler(void)
{
  //--------------------------TX-----------------------
  if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
  {
    GREEN_LED_ON();//G
    idleDMAcycles=0;
    USART2->DR =((uint16_t)(U2tx[p2tx++]))&(uint16_t)0x00FF;
    if(--u2txleft==0)USART_ITConfig(USART2,USART_IT_TC,DISABLE);
    return;
  }//TX
  //-----------------check errors-----------------
  ITStatus s1= USART_GetITStatus(USART2,USART_IT_IDLE);   // Idle line detection interrupt
  ITStatus s2= USART_GetITStatus(USART2,USART_IT_ORE_RX); // OverRun Error interrupt if the RXNEIE bit is set
  ITStatus s3= USART_GetITStatus(USART2,USART_IT_ORE_ER); // OverRun Error interrupt if the EIE bit is set  
  ITStatus s4= USART_GetITStatus(USART2,USART_IT_NE);     //   Noise Error interrupt
  ITStatus s5= USART_GetITStatus(USART2,USART_IT_FE);     //   Framing Error interrupt
  ITStatus s6= USART_GetITStatus(USART2,USART_IT_PE);     //  Parity Error interrupt
  
  if(s2)USART_ReceiveData(USART2);
}
//------------------------------------------------------------------------------
//обработка вход€щей команды
static void process_cmd(ICMD cmd) 
{
  uint8_t txlen=0;
  int dis=0;
  //----------select command------------
      
  switch(cmd)
  {
   case IC_GET_LVER:
      U2tx[0]=(uint8_t)2;//2=LScan2 version
      txlen=1;
      break;
   case IC_SET_USB_DISABLE:
      if(ICmode==IM_USB)Soft_Disconnect_USB_Device(&USB_OTG_dev);//usb core device;
      ICmode=IM_WLAN;
      break;
   case IC_GET_STREAM_PAK_SIZE://isoc packet size
      U2tx[0]=(uint8_t)(IsocOutPacketSize/8);
      txlen=1;
      break;
   case IC_GET_STREAM_BUF_SIZE://buff size in packets
      U2tx[0]=(uint8_t)((int)LS_BUF_SIZE/(int)IsocOutPacketSize);
      txlen=1;
      break;
   case IC_GET_BUF_FREE_8://free buff size in bytes/8
      dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;
      *((uint16_t*)U2tx)=(uint16_t)(dis/8);
      txlen=2;
      break;
   case IC_GET_BUF_FREE_PACKS://free in buff (in packets)
      dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;
      U2tx[0]=(uint8_t)(dis/(int)IsocOutPacketSize);  
      txlen=1;
      break;
   case IC_GET_BUF_FREE_256://free in buff(256-bytes parts)
      dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;//буфер свободен
      U2tx[0]=(uint8_t)(dis/256); //частей 
      txlen=1;
      break;
  //-------------------  
  case IC_NO:
  default:
      dis=BFTRING(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
      if(LScanStatus==STATUS_STOP && dis==0)dis=(int)LS_BUF_SIZE;
      U2tx[0]=(uint8_t)(dis/(int)IsocOutPacketSize);  
      txlen=1;
      break;
   }//switch(cmd)
  
  if(txlen==0)return;
  
  if(USART_GetFlagStatus(USART2,USART_FLAG_TXE)!=RESET)
  {
    p2tx=0;
    USART2->DR =(uint16_t)(U2tx[p2tx++])&(uint16_t)0x00FF;
   if(u2txleft=--txlen)USART_ITConfig(USART2,USART_IT_TC,ENABLE);
  }
  else 
  {
    U2TX_Stream->CR &= ~(uint32_t)DMA_SxCR_EN;
    while (U2TX_Stream->CR & (uint32_t)DMA_SxCR_EN);
    U2TX_Stream->M0AR =  (uint32_t)U2tx;
    U2TX_Stream->NDTR = (uint32_t)txlen;// len
    USART_DMACmd(USART2, USART_DMAReq_Tx,ENABLE);
    DMA_ClearFlag(U2TX_Stream,U2TX_DMA_FLAG_HTIF|U2TX_DMA_FLAG_TCIF|U2TX_DMA_FLAG_FEIF|U2TX_DMA_FLAG_TEIF);
    DMA_ITConfig(U2TX_Stream, DMA_IT_TC|DMA_IT_TE, ENABLE);//прерывание по завершению 
    U2TX_Stream->CR |=  (uint32_t)DMA_SxCR_EN;//start tx req
    u2txleft=0;
  }
}
//------------------------------------------------------------------------------
void UDMA_transfer(DMA_Stream_TypeDef* DMAy_Streamx,uint32_t blocks,uint32_t opt_buffer_addr)
{
  DMAy_Streamx->CR &= ~(uint32_t)DMA_SxCR_EN;
  while (DMAy_Streamx->CR & (uint32_t)DMA_SxCR_EN);
  if(opt_buffer_addr)DMAy_Streamx->M0AR=opt_buffer_addr;
  DMAy_Streamx->NDTR = blocks;// len
  DMA_ITConfig(DMAy_Streamx, DMA_IT_TC|DMA_IT_TE, ENABLE);//прерывание по завершению|ошибке 
  DMAy_Streamx->CR |=  (uint32_t)DMA_SxCR_EN;//start req
}
//------------------------------------------------------------------------------
void UDMA_stop(DMA_Stream_TypeDef* DMAy_Streamx)
{
  DMAy_Streamx->CR &= ~(uint32_t)DMA_SxCR_EN;
  while (DMAy_Streamx->CR & (uint32_t)DMA_SxCR_EN);
  DMA_ITConfig(DMAy_Streamx, DMA_IT_TC|U2TX_DMA_IT_TEIF, DISABLE);//прерывание по завершению 
}
//------------------------------------------------------------------------------
//корректна€ остановка DMA rx
static void StopU2RXstream()
{
  DMA_ITConfig(U2RX_Stream, DMA_IT_TC|DMA_IT_TE|DMA_IT_FE|DMA_IT_HT, DISABLE);
  DMA_Cmd(U2RX_Stream,DISABLE);//rx
  while (U2RX_Stream->CR & (uint32_t)DMA_SxCR_EN);
  
  USART_DMACmd(USART2, USART_DMAReq_Rx,DISABLE);
    
  DMA_ClearITPendingBit(U2RX_Stream,U2RX_DMA_IT_TEIF|U2RX_DMA_IT_FEIF);
}
//------------------------------------------------------------------------------
//перезагрузка DMA rx.¬ случае зависани€ на заданное врем€ произвести перезагрузку
//во избежание потери последовательности заполнени€ потокового буфера.
static void StartU2RXstream()
{
  UDMA_stop(U2RX_Stream);
  //------Peripheral DMA init USART1 RX=DMA2_stream5 Channel4---------------
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART2->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U2rx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize =(uint32_t)(2*sizeof(ICP));//NDTR in data unit
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//FIFO->buf->memcpy
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;//DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;//8 bytes
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(U2RX_Stream, &DMA_InitStructure);
  //-------------IT---------------------
  DMA_ClearITPendingBit(U2RX_Stream,U2RX_DMA_IT_HTIF|U2RX_DMA_IT_TCIF/*|U2RX_DMA_IT_FEIF*/|U2RX_DMA_IT_TEIF);
  
  USART_DMACmd(USART2, USART_DMAReq_Rx,ENABLE);
  DMA_ITConfig(U2RX_Stream,
    DMA_IT_HT//прерывание по 1/2 FIFO
  | DMA_IT_TC //прерывание по заполнению
  | DMA_IT_TE //прерывание по ошибке 
  //| DMA_IT_FE //прерывание по ошибке FIFO
    ,ENABLE);
  
  DMA_ClearFlag(U2RX_Stream,U2RX_DMA_FLAG_HTIF|U2RX_DMA_FLAG_TCIF/*|U2RX_DMA_FLAG_FEIF*/|U2RX_DMA_FLAG_TEIF);
  //---------------------------------------------
  DMA_Cmd(U2RX_Stream,ENABLE);//start rx
}
//------------------------------------------------------------------------------
//ѕроизводитс€ проверка DMA на зависание и перевод в режим ожидани€ (фон.режим)
void U2timerProc(void)//* ctx)
{
  if((U2RX_Stream->CR & (uint32_t)DMA_SxCR_EN)==0)//DMA выключен !
  {
    idleDMAcycles=0;
    if(U2DMA_timer)if(DeleteSystemTimer(&U2DMA_timer)==FALSE)while (1);
    return ;
  }//DMA RX выключен
  
  if(++idleDMAcycles >= (uint32_t)TRX_IDLE_SEC)//секунд 
  {
    RED_LED_ON();GREEN_LED_ON();//индикаци€ перезагрузки
    idleDMAcycles=0;
    //--------------
    if(ICmode == IM_WLAN)
    {
      if(U2DMA_timer)if(DeleteSystemTimer(&U2DMA_timer)==FALSE)while (1);
      //----------------------DeleteSystemTask(&U2DMA_timer);
      StopU2RXstream();
      USART_ReceiveData(USART2);
      StartU2RXstream();//перезагрузка DMA rx
        
      UDMA_stop(U2TX_Stream);
      USART_DMACmd(USART2, USART_DMAReq_Tx,DISABLE);
      
      Connect_FS_USB_Device(&USB_OTG_dev);
      ICmode=IM_USB;
      return ;
    }
    //--------------
    StopU2RXstream();
    USART_ReceiveData(USART2);
    StartU2RXstream();//перезагрузка DMA rx
    //--------------перезагрузка DMA tx-----------------
    UDMA_stop(U2TX_Stream);
    USART_DMACmd(USART2, USART_DMAReq_Tx,DISABLE);
    //re-enable
    DMA_ClearFlag(U2TX_Stream,U2TX_DMA_FLAG_HTIF|U2TX_DMA_FLAG_TCIF|U2TX_DMA_FLAG_FEIF|U2TX_DMA_FLAG_TEIF);
  }
  
  return ;
}
//------------------------------------------------------------------------------