
#include "spi_wlan.h"

extern IMODE ICmode;//тип соединения
extern uint8_t LScanStatus;
extern volatile int freesize;
extern uint32_t errors;
extern uint8_t* StreamOverbuf;

#pragma data_alignment = 4   
volatile uint32_t val;//for rx
#pragma data_alignment = 4   
volatile uint32_t val2;//for tx

static volatile uint32_t _lastRxTicks=0;//последнее событие

static void SPI1_DMA_RxConfig();

static void SPI1_DMA_TxConfig();

static int _check_for_SPI1_state(void* pArgs);

static inline void tx();

static volatile int right=0;//сколько загружалось
static volatile int tlen=0;//static bool start=true;

extern volatile uint8_t stream_mode;

extern volatile int cnt_icp;
extern volatile bool Exit_Proccess_Player;
static uint32_t init_stamp=0;
//________________________________________________________________________________________________________//
static void SPI1_NVIC_Configuration(void)
{
  //__enable_irq();
  
  NVIC_InitTypeDef NVIC_InitStructure;

  /*NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)SPI1_IRQn_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)SPI1_IRQn_PRIO_SUB;
  NVIC_Init(&NVIC_InitStructure);*/

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;//RX
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)SPI1_DMA_IRQn_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)SPI1_DMA_IRQn_PRIO_SUB;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //return;
  //NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;//TX
  //NVIC_Init(&NVIC_InitStructure);
  
  NVIC_DisableIRQ(SPI1_CS_EXTI_IRQ); //return;
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_CS_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)1;//SPI1_IRQn_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)3;//SPI1_IRQn_PRIO_SUB;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}
//________________________________________________________________________________________________________//
void SPI_WLAN_init(bool newThread){
  
  //static uint32_t init_ck=0;

  //=======reset sequence???=======
  //if(!newThread && _get_timestamp-init_stamp<=500){ //3x less than 500ms=reset controller
  //  if(++init_ck>=3){
  //    init_ck=0;Exit_Proccess_Player=true;
  //    NVIC_SystemReset();
  //  }
  //}
  //else 
  //init_ck=0;
  
  init_stamp=_get_timestamp;
  ///==============================
  //SCK, MISO, MOSI PB3 PB4 PB5 
  if(ICmode==IM_WLAN)ICmode=IM_NO;
  
  //DMA channel 3 stream 0,2 = SPI1 RX; stream 3,5 = TX
  
  int i=168000;//~1ms
  while (DMA2_Stream2->CR & DMA_SxCR_EN && i--) {};
  // Disable the selected DMAy Streamx by clearing EN bit
  DMA2_Stream2->CR &= ~(uint32_t)DMA_SxCR_EN;
  DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2);
  //DMA2_Stream2->CR &= ~(uint32_t)DMA_SxCR_EN;//DMA_DeInit(DMA2_Stream2);
  i=168000;//~1ms
  while (DMA2_Stream3->CR & DMA_SxCR_EN && i--) {};
  DMA2_Stream3->CR &= ~(uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream3, DISABLE);
  //i=168000;//~1ms
  //while (DMA2_Stream3->CR & DMA_SxCR_EN && i--) {};
  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3 );
  //DMA_DeInit(DMA2_Stream3);
  SPI1->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
  SPI_I2S_DeInit(SPI1);
  
  stream_mode=0;
 
  GPIO_InitTypeDef gpio;
  GPIO_StructInit(&gpio);
 
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  
  gpio.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//CLK|MISO|MOSI
  gpio.GPIO_Mode = GPIO_Mode_AF;//GPIO_Mode_AF;
  GPIO_Init(GPIOB, &gpio);
  
  gpio.GPIO_Pin=SPI1_CS_PIN;//CHIP SELECT, REBOOT
  gpio.GPIO_Mode=GPIO_Mode_IN;
  gpio.GPIO_PuPd=GPIO_PuPd_DOWN;//если не подключено 
  GPIO_Init(SPI1_CS_PORT, &gpio);

  gpio.GPIO_Pin =SPI1_HANDSHAKE_PIN;
  gpio.GPIO_Mode=GPIO_Mode_OUT;//HANDSHAKE == UP
  gpio.GPIO_OType=GPIO_OType_PP;
  gpio.GPIO_PuPd= GPIO_PuPd_NOPULL;
  GPIO_Init(SPI1_HANDSHAKE_PORT, &gpio);
  SPI1_HAND_LOW;
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1); //CLK
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1); //MISO TX
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1); //MOSI RX

  SPI1_NVIC_Configuration();
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);// 84 MHz
  //-------------------------------------
  SPI_InitTypeDef spistr;
  SPI_StructInit(&spistr);
  spistr.SPI_Direction =SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Rx; //полный дуплекс
  spistr.SPI_DataSize = SPI_DataSize_8b; // передаем по 8 бит
  spistr.SPI_CPOL = SPI_CPOL_Low; // Полярность и
  spistr.SPI_CPHA = SPI_CPHA_1Edge; // фаза тактового сигнала
  spistr.SPI_NSS = SPI_NSS_Soft;
#ifdef SPI1_USE_HARD_CS
  //spistr.SPI_NSS = SPI_NSS_Soft;//SPI_NSS_Hard;
#else 
  //spistr.SPI_NSS = SPI_NSS_Soft;SPI_CR1_BR
#endif  
  //spistr.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//SPI1_PRESCALER_MASTER_MODE; // Предделитель SCK
  spistr.SPI_FirstBit = SPI_FirstBit_MSB; // Первым отправляется старший бит
  spistr.SPI_Mode = SPI_Mode_Slave;
  
#ifdef SPI1_ENABLE_CRC  
  spistr.SPI_CRCPolynomial = 0x07; // Задаем значение параметра CRCPolynomial
  SPI_CalculateCRC(SPI1,ENABLE); //Включаем расчёт CRC
#else
  SPI_CalculateCRC(SPI1,DISABLE);
#endif
  //--------------------------------------
  SPI_Init(SPI1,&spistr);
  SPI1->CR1 |= SPI_CR1_SPE;               //Enable SPI*/
  if(newThread)EnterCriticalSection;//NOT IN IT !!!
    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  
  SPI1_DMA_TxConfig();
  SPI1_DMA_RxConfig();
  //ICmode=IM_WLAN;
  if(newThread)LeaveCriticalSection;

  //char* pbuf=(char*)StreamBuf;
  
  //if(newThread)CreatePeriodicTaskWithFlags(_check_for_SPI1_state,NULL,SPI1_DMA_HUNG_DETECT_TICKS,TMMRAM_STACK_Msk);
  
  //if(newThread)CreateThread((TFUNC*)&_check_for_SPI1_state,0,TM_PRIV|TM_PSP|TMMRAM_STACK_Msk,256);//TM_PRIV|TM_PSP,0);
  //return;
  
  SYSCFG_EXTILineConfig(SPI1_CS_EXTI_PORT, SPI1_CS_EXTI_PIN);
  EXTI_ClearITPendingBit(SPI1_CS_EXTI_LINE);
  
  EXTI_InitTypeDef exti;
  EXTI_StructInit(&exti);
  exti.EXTI_Line =SPI1_CS_EXTI_LINE;
  exti.EXTI_Mode = EXTI_Mode_Interrupt;    
  exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //фронт,спад
  exti.EXTI_LineCmd = ENABLE;    // вкл
  EXTI_Init(&exti);
  
}
//________________________________________________________________________________________________________//
static void RX_disable(){
  SPI1_HAND_LOW;//NOT READY
  
  
  
  stream_mode=0;
  right=tlen=0;
  Exit_Proccess_Player=true;
  if(ICmode==IM_WLAN)ICmode=IM_NO;
  
  //int i=168000;//~1ms
  //while (DMA2_Stream2->CR & DMA_SxCR_EN && i--) {};
  DMA2_Stream2->CR &= ~(uint32_t)DMA_SxCR_EN;
  // SPI DMA requests 
  //SPI1->CR2 &=~(uint16_t)SPI_I2S_DMAReq_Rx;
  
  DMA2_Stream2->NDTR = 0;
  
  DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2);

  SPI1->CR1 |= SPI_NSSInternalSoft_Set;
  //val2=0x594452;//RDY
}

static void RX_enable(){
  stream_mode=0;right=tlen=0;
  SPI1->CR1 &= SPI_NSSInternalSoft_Reset;
  DMA2_Stream2->NDTR = 4;
  //val2=0x594452;
  // Enable the selected SPI DMA requests 
  //SPI1->CR2 |= SPI_I2S_DMAReq_Rx;
  DMA2_Stream2->CR |= (uint32_t)DMA_SxCR_EN;
  SPI1_HAND_HIGH;//READY
  ICmode=IM_WLAN;
}
//________________________________________________________________________________________________________//
void SPI1_CS_EXTI_IRQHandler()
{
  static uint32_t t=0;
  static int n=0;
  
  ITStatus CS_IT=EXTI_GetITStatus(SPI1_CS_EXTI_LINE);
  
  
  
  if(CS_IT){
    EXTI_ClearFlag(SPI1_CS_EXTI_LINE);
    
    if(SPI1_CS_PORT->IDR & SPI1_CS_PIN){ //CS HIGH
      //SPI1_HAND_LOW;
      //SPI1->CR1 |= SPI_NSSInternalSoft_Set;
      SPI1->CR1 |= SPI_NSSInternalSoft_Set;
      SPI1_HAND_LOW;//NOT READY
      right=tlen=stream_mode=0;
      Exit_Proccess_Player=true;
      if(ICmode==IM_WLAN)ICmode=IM_NO;
      DMA2_Stream2->CR &= ~(uint32_t)DMA_SxCR_EN;
      SPI1->CR2 &=~(uint16_t)SPI_I2S_DMAReq_Rx;
      //DMA2_Stream2->NDTR = 4;
      DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2);

      t=_get_timestamp-t;
      if(t<15){ //reset/reboot sequence
        n++;
      }
      else {
       if(n){
          if(n<2){ //reset spi bus(DMA) 1 short
            n=0;
            NVIC_DisableIRQ(SPI1_CS_EXTI_IRQ);
            stream_mode=0;right=tlen=0;
            val2=0x594452;//RDY
            Exit_Proccess_Player=true;
            rdPtr=wrPtr=StreamBuf;cnt_icp=0;freesize=STREAMBUF_SIZE;
            SPI_WLAN_init(false);
          }
          else { //reboot 2 short
            n=0;
            GREEN_LED_ON();BLUE_LED_ON();RED_LED_ON();ORANGE_LED_ON(); 
            rdPtr=wrPtr=StreamBuf;cnt_icp=0;freesize=STREAMBUF_SIZE;
            NVIC_SystemReset();
          }
        }
       n=0;
      }
    
      t=_get_timestamp;
      return;
    }
    //------------------------------------------------
    _lastRxTicks=_get_timestamp;
    right=tlen=stream_mode=0;
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2);
    SPI1->CR1 &= SPI_NSSInternalSoft_Reset;
    DMA2_Stream2->NDTR = 4;
    // Enable the selected SPI DMA requests 
    SPI1->CR2 |= SPI_I2S_DMAReq_Rx;
    DMA2_Stream2->CR |= (uint32_t)DMA_SxCR_EN;
    //val2=0x594452;// RDY (TX)
    SPI1_HAND_HIGH;//READY
    ICmode=IM_WLAN;
    
    //SPI1->CR1 &= SPI_NSSInternalSoft_Reset;

  }
}
//________________________________________________________________________________________________________//
void SPI_WLAN_deinit(){
  if(ICmode==IM_WLAN)ICmode=IM_NO;
  int i=168000;//~1ms
  while (DMA2_Stream3->CR & DMA_SxCR_EN && i--) {};
  DMA_Cmd(DMA2_Stream3, DISABLE);
  i=168000;//~1ms
  while (DMA2_Stream3->CR & DMA_SxCR_EN && i--) {};
  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3 );
  DMA_DeInit(DMA2_Stream3);
  SPI1_HAND_LOW;
  SPI1->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
  SPI_I2S_DeInit(SPI1);
}
//________________________________________________________________________________________________________//
//проверка состояния
static int _check_for_SPI1_state(void* pArgs){
  static uint8_t CS=1;
  static bool deinit=false;
 
  while(1){

    RED_LED_OFF(); BLUE_LED_OFF();
    
    if(SPI1_CS_PORT->IDR & SPI1_CS_PIN){ //CS HIGH
      CS=1;
      SPI1_HAND_LOW;
      if(!deinit){
        deinit=true;stream_mode=0;
        init_stamp=_get_timestamp;
        SPI_WLAN_deinit();stream_mode=0;
      }
    }
    //---wait CS LOW---
    if(CS)
    { 
      SPI1_HAND_LOW;
      while(SPI1_CS_PORT->IDR & SPI1_CS_PIN){ //CS HIGH
        if(_get_timestamp-init_stamp>=400){
          Exit_Proccess_Player=true;
          NVIC_SystemReset();
        }
        Sleep(10);
      } 
      
      //REINIT
      deinit=false;
      CS=0;right=tlen=0;
      Exit_Proccess_Player=true;
      ICmode=IM_WLAN;
      SPI_WLAN_init(false);
    }
    //обнаружение простоя
    if(SPI1_DMA_HUNG_DETECT_TICKS>0){
      uint32_t _sysTks=_get_timestamp;
      if(_sysTks-_lastRxTicks>=SPI1_DMA_HUNG_DETECT_TICKS){
        SPI1_HAND_LOW;
        RED_LED_ON(); 
        SPI_WLAN_init(false);
      }
    }
    
    Sleep(50);
  }
  
}
//________________________________________________________________________________________________________//
/*
память буфера должна быть в MM_RAM!!! - MM_RAM_START  == начало области памяти(см.datasheet)
*/
static void SPI1_DMA_RxConfig()
{
  SPI1_HAND_LOW;
  
  DMA_InitTypeDef DMAstruct;
  DMA_StructInit(&DMAstruct);
  //------------------------------------------------------
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SPI1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&val;//загружаем в int
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 4;//2 x sizeof(int); //длина приёма
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  //DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_VeryHigh;//DMA_Priority_Low;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  //FIFO не используется,т.к. нет флага для 8 бит.
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
  
  SPI1->CR1 |= SPI_NSSInternalSoft_Set;//OFF
  DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2 );
  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC/*|DMA_IT_TE*/, ENABLE);
  
  
  //DMA_FlowCtrl_Peripheral сбивает NDTR !!!
  //SPI1->CR1 &= SPI_NSSInternalSoft_Reset;
  // Enable the selected SPI DMA requests 
  SPI1->CR2 |= SPI_I2S_DMAReq_Rx;
  _lastRxTicks=0;//_get_timestamp;
  //DMA2_Stream2->CR |= (uint32_t)DMA_SxCR_EN;
  //SPI1_HAND_HIGH;//READY
}
//________________________________________________________________________________________________________//
static inline void tx(){
  if(DMA2_Stream2->NDTR==0)DMA2_Stream2->NDTR =4;
  DMA2_Stream2->CR |= (uint32_t)DMA_SxCR_EN; //RX enable
  SPI1_HAND_HIGH;
  DMA2_Stream3->CR |= (uint32_t)DMA_SxCR_EN; //TX enable
}
/*
void SPI1_DMA_TX_IRQHandler(void){
  SPI1_HAND_LOW;
  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3 );
  rx();
}*/
//________________________________________________________________________________________________________//
//RX=DMA2 STREAM2 Channel3 TX=DMA2 STREAM3 Channel3 , work with StreamBuf,wrPtr,rdPtr,STREAMBUF_SIZE, 
void SPI1_DMA_RX_IRQHandler(void){
  static int prevCmd=0;
  int rr;

  SPI1_HAND_LOW;

  //обязательно!!!   
 // DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3 );//TX clear
  DMA2->LIFCR|=(uint32_t)((DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|DMA_FLAG_TEIF3|DMA_FLAG_HTIF3|DMA_FLAG_TCIF3) & (uint32_t)0x0F7D0F7D);
  if(DMA2->LISR & DMA_FLAG_TCIF2)
  {
    //DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
    DMA2->LIFCR|=((uint32_t)DMA_FLAG_TCIF2 & (uint32_t)0x0F7D0F7D);
    
    //if(errors)return;
    
#ifdef SPI1_ENABLE_CRC
    if(SPI1->SR & SPI_SR_CRCERR){ //ERROR? retry
      
      SPI1->SR&=~(uint16_t)SPI_SR_CRCERR;
      
      sprintf((char*)&val2,"CRC");
      
      tx();
      
      _lastRxTicks=_get_timestamp;
      RED_LED_ON(); 
      return;
    }
#endif
  
    //--------------no error------------------
    if(stream_mode){
      
      int f=*(&freesize)-SPI_DMA_BLOCK_SIZE;
      //*(&freesize)=f;
      
      
      /*if(f>=SPI_DMA_BLOCK_SIZE){//SPI_DMA_STRT_FREESZ){
        
        volatile uint8_t* pw=*(&wrPtr)+SPI_DMA_BLOCK_SIZE;
        if(pw>=StreamOverbuf)pw-=STREAMBUF_SIZE;//pw=StreamBuf;
        *(&wrPtr)=pw;
        *(&freesize)=f;
        
        DMA2_Stream2->NDTR = (uint32_t)SPI_DMA_BLOCK_SIZE;
        DMA2_Stream2->M0AR = (uint32_t)pw;
        DMA2_Stream2->CR |= (uint32_t)DMA_SxCR_EN;
        SPI1_HAND_HIGH; 
      }
      else*/ stream_mode=2;
      
      PlayFlag=1;LScanStatus=STATUS_PLAY;
      
      _lastRxTicks=_get_timestamp; BLUE_LED_ON(); 
      return;
    }
    else
    if(prevCmd==WCMD_STREAM){
      prevCmd=0;
      //PlayFlag=0;
      SetRGB(0,0,0);ICmode=IM_WLAN;
      /*
      //rdPtr=wrPtr=StreamBuf;cnt_icp=0;freesize=STREAMBUF_SIZE;
      DMA2_Stream2->M0AR = (uint32_t)wrPtr;
      DMA2_Stream2->NDTR = (uint32_t)SPI_DMA_BLOCK_SIZE;
      stream_mode=1;
      PlayFlag=0;rdPtr=wrPtr=StreamBuf;cnt_icp=0;freesize=STREAMBUF_SIZE;
      tx();
      */
      stream_mode=1;
      rdPtr=wrPtr=StreamBuf;cnt_icp=0;freesize=STREAMBUF_SIZE;
      DMA2_Stream2->M0AR = (uint32_t)StreamBuf;
      DMA2_Stream2->NDTR = (uint32_t)SPI_DMA_BLOCK_SIZE;//STREAMBUF_SIZE;
      //DMA2_Stream2->CR &= ~((uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE));// Disable all DMA interrupts
      DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_EN /*| DMA_SxCR_CIRC*/);
      PlayFlag=0;rdPtr=wrPtr=StreamBuf;cnt_icp=0;freesize=STREAMBUF_SIZE;
      SPI1_HAND_HIGH;//READY
      tx();
      _lastRxTicks=_get_timestamp; BLUE_LED_ON(); 
      
      return;
    }
    else if(tlen){
      right=tlen;tlen=0;
      uint32_t memPtr=(uint32_t)wrPtr;

      DMA2_Stream2->M0AR = memPtr;//wrPtr;
      DMA2_Stream2->NDTR = (uint32_t)right;
      
      tx();
      
      _lastRxTicks=_get_timestamp; BLUE_LED_ON(); 
      return;
    }
    else if(right){
      wrPtr+=right;if(wrPtr>=StreamOverbuf)wrPtr=StreamBuf;

      rr=right;
      freesize-=rr;

      if(freesize<0){ //check error
        RED_LED_ON();ORANGE_LED_ON();
        errors|=ERR_FREE_NEGATIVE;
      };

      rr=freesize;
      val2=right|(rr<<16);//значение freesize в старшем полуслове
      right=0;
        
      DMA2_Stream2->M0AR = (uint32_t)&val;
      DMA2_Stream2->NDTR =4;
 
      tx();

      _lastRxTicks=_get_timestamp; BLUE_LED_ON(); 
      return;
    }
        
    //пришла команда======================>
    switch(val&0xFF){
    case WCMD_PREP_DMA:
      rr=(val>>8);//значение счётчика
      if(freesize>=sizeof(ICP) && rr<0x10000){ //16 бит==корректно
        //высчитываем количество, которое можем принять
        if(rr>freesize)rr=freesize;
        
        if(rr+wrPtr>=StreamOverbuf){
          rr=StreamOverbuf-wrPtr;
        }
        
        rr&=(uint32_t)~7;//8 align
        
        tlen=rr;
        
        //загрузка на следующем
        val2=rr|((freesize-rr)<<16);//значение freesize в старшем полуслове

        tx();
    
        _lastRxTicks=_get_timestamp; BLUE_LED_ON(); 
        return;
      }
      //ERROR left<0 (not critical)
      val2=(freesize<<16);//значение freesize в старшем полуслове
      right=0;
      DMA2_Stream2->M0AR = (uint32_t)&val;
     break;
    case WCMD_FREE:val2=freesize;break;
    case WCMD_BUFSIZE:val2=STREAMBUF_SIZE;break;
    case WCMD_SAMPLESIZE:val2=4;break;
    case WCMD_STREAM_BLOCKSIZE:val2=SPI_DMA_BLOCK_SIZE;break;
    case WCMD_STREAM:
      Exit_Proccess_Player=true;PlayFlag=0;SetRGB(0,0,0);ICmode=IM_WLAN;
      sprintf((char*)&val2,"ok");
      prevCmd=WCMD_STREAM;
      break;
    case WCMD_PING:val2=0x594452;break;//RDY
    //case 0:if(!stream_mode)val2=0x594452;break;//RDY
    }
    
    tx();
    //--------------
    _lastRxTicks=_get_timestamp;
    BLUE_LED_ON(); 
    return;
  }//if(DMA2->LISR & DMA_FLAG_TCIF2)
  //==========================================
  // ERROR !!! 
  tx();
  
  RED_LED_ON(); 

  _lastRxTicks=_get_timestamp;
}
//________________________________________________________________________________________________________//
/*
память буфера должна быть в MM_RAM!!! - MM_RAM_START  == начало области памяти(см.datasheet)
*/
static void SPI1_DMA_TxConfig()
{
  
  DMA_InitTypeDef DMAstruct;
  DMA_StructInit(&DMAstruct);
  //------------------------------------------------------
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SPI1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&val2;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);
  
  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3 );
  
  SPI1->CR2 |=SPI_I2S_DMAReq_Tx;

}
//________________________________________________________________________________________________________//

