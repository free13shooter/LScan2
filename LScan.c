/**
  ******************************************************************************
  * @file    LScan.c 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    11-May-2014
  * @brief   LScan functions and interrupts program body
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "LScan.h"
#include "sdcard.h"
#include "spi_wlan.h" //STM32 <-> ESP32 WiFi net

uint32_t errors=0;

uint8_t LScan_CMD=LSCAN_CMD_STOP;//�������

__IO uint16_t ADC1ConvertedValue[2];//������ ADC (DMA)

uint16_t R_Ctrl,G_Ctrl,B_Ctrl;//���-�������� ��������� ������� �����

/* Private macro -------------------------------------------------------------*/
// sound
extern uint8_t IsSoundConfigured;
extern float SoundGain;//��������(�������� ��������� ��� ������������ ���)
/* Private variables ---------------------------------------------------------*/
#pragma data_alignment = 4   
uint8_t StreamBuf[STREAMBUF_SIZE];//����� ������ ��������������� FIFO.
#pragma data_alignment = 4   
uint8_t* StreamOverbuf=StreamBuf+STREAMBUF_SIZE;
#pragma data_alignment = 4   
volatile uint8_t* wrPtr;
#pragma data_alignment = 4   
volatile uint8_t* rdPtr;
#pragma data_alignment = 4   
volatile uint8_t  PlayFlag;//���� ���������������
#pragma data_alignment = 4   
volatile uint8_t stream_mode=0;//����� DMA spi wlan

//.................................
RCC_ClocksTypeDef RCC_Clocks;
uint16_t lasersOffCnt;
bool FullSysReset;
//-------------------------------------------------------------	

//������� ��� ������
int Colors_PWM_Freq=COLORS_PWM_FREQ_MHZ;//1-168
//������ ��������,%
int R_Tresh=2;
int G_Tresh=3;
int B_Tresh=2;

//������ ������ ������ � ���������� ��������� �������������
int RGB_on_shift=((int)OPT_DISCR*75/100);int RGB_off_shift=((int)OPT_DISCR*75/100);   //TIM9=R,TIM10=G,TIM11=B

//����������� �������� ����� �� �������,����������� ����������������� ������:
float RGB_on_k=0.5f,RGB_off_k=0.5f;
float Linearity=0.5f;//������� �������� ������[1.01;100] 1.01-�������� �������,������-�������� �����

#ifdef USE_SPD_CORRECTOR
__IO int sampleMult;//��������� ��������,��������
#endif

//��������������� ������� ���������� ��� � ����������� �� ��������� ����������  
uint16_t R_PWM[256];
uint16_t G_PWM[256];
uint16_t B_PWM[256];
//-------------------------------------------------------------
//������� �������� ���������/������� �������,�������������� ��� ������������ ������� ��������
//uint16_t smplPeriod[LIM_DISCR_MS+1];//������ ��� ������ ������� �������������
uint16_t offsetRGBon[256];
uint16_t offsetRGBoff[256];
//uint8_t ddpss[256];//��������� �������� ��������� ��� dpss-������ � ���������
//-------------------------------------------------------------	
//���������� �������� ���������
int R_power=100;
int G_power=100;
int B_power=100;

//��������� ��� ������-�������� �������� �������� ������� � ����

//����������� ���������� ��� ��� ������ � ���������(0-100):
int R_pwm_min=0;//1;
int G_pwm_min=0;//3;
int B_pwm_min=0;//1;
/*
������������ ���������� ��� ��� ������ � ���������(0-100):
����� ���������� ����������� ��������� ��� ���������� �������
��� ����������� ��������� �������.
*/
int R_pwm_max=100;//71;//100;
int G_pwm_max=100;
int B_pwm_max=100;//36;//100; ����� ������� �����,��� ���������� �������������

//-------------------------------------------------------------	
OPT_V opt={0.0f,0.0f};//���������� ��������� �����������
//_____________________

__IO uint32_t SampleDiscrInMs=0;//������� ������������� ��� 1 ��-������


uint8_t LScanStatus=STATUS_STOP;

__IO uint16_t X=2048,Y=2048; //������� ���������� 
__IO uint8_t R=0,G=0,B=0;	  //������� ����� 

//uint16_t mel8notes[16]={2000,90,1200,90,1500,100};
//system core clock in system_stm32f4xx.c
//extern bool IsDeviceConnected;

//APB1,APB2 ������� �������� �������� �� 2!!!(PCLK1x2,PCLK2x2)

bool SDCardMounted=false;          /* File system object logical drive mounted*/
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
volatile IMODE ICmode=IM_WLAN;//��� ����������
//----------------------------------------------------------------------------//
SERVO_PAR wndServo={50,227,178};
bool Power=false;//��������� �������
uint16_t WndGrad=0;//0=���� ������� !0=�������
SystemTimer* Power_timer=NULL;//��������� ������ ���������� �� ������������
unsigned char ServoOffPwrCycles=0;//0==��������� ���������� ����� ����
//----------------------------------------------------------------------------//
FATFS* FS_SDIO_disk;
FATFS* FS_SPI_disk;

static IMODE oldICmode=IM_NO;//���������� �����
//----------------------------------------------------------------------------//
static uint8_t newR_pwm=0;
static uint8_t newG_pwm=0;
static uint8_t newB_pwm=0;
//----------------------------------------------------------------------------//
extern uint8_t button[9];//������ ������ � ����-������

volatile TYPEMODEMASK TMMask=0;//TM_FILE_RANDOM;    //����� �������� ������� ��������������� ������,LScan.c

//playtimer variables
volatile int freesize=STREAMBUF_SIZE;
volatile int cnt_icp=0;
volatile uint32_t rdy_ilda=0; 
//==================================================================================================
void LScan_Init()
{
  PlayFlag=0;
  
#ifdef USE_SPD_CORRECTOR
  sampleMult=100;
#endif
     
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//SYSCFG APB clock must be enabled to get write access to SYSCFG_EXTICRx
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);//random number generator
  RNG_Cmd(ENABLE);
  /* Enable AHB clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|
                         RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);
  
  //------------	
  LSCAN2_LEDInit(LEDGREEN);
  LSCAN2_LEDInit(LEDRED);
  LSCAN2_LEDInit(LEDORANGE);
  LSCAN2_LEDInit(LEDBLUE); 
  
  //---------SETUP System TIMER--------------
  RCC_ClocksTypeDef RCC_Sys;
  RCC_GetClocksFreq(&RCC_Sys);
  
  rdPtr=wrPtr=StreamBuf;cnt_icp=0;freesize=STREAMBUF_SIZE;
  
  //------------run SPI3 to ESP32 wlan intercommunication----------
  SPI_WLAN_init(true);
  
  //RED_LED_ON();
  //return;
  //------------SETUP USB DEVICE----------------
  //USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CUSTOM_cb,&USR_cb); // usbd_custom_core.c
  
  //MEMS_DPM_MIC_Config();
#ifndef DISCOVERY_BOARD
  Buttons_config();
#endif  
    
  //----SETUP temp. control ADC+FANs PWM----------
//#ifndef DISCOVERY_BOARD
  ADC_Reg_Config();
//#endif  
  //------------SETUP POWER PIN-----------------
  Power_pin_config();//�������
  
  FANs_Servo_PWM_config();
  Wnd_CLOSE();
  //-------------SETUP SOUND--------------------
  Sound_Config();
  SetSoundGain(0.1f);
  //BeepMelody(mel8notes,3);
  
  //----UART---- 
  //UART2_init();
  //------------SETUP COLORS PWM----------------
  Calculate_PWM_Table(RED_COLOR);
  Calculate_PWM_Table(GREEN_COLOR);
  Calculate_PWM_Table(BLUE_COLOR);//���������� ������� ����������
  
  Colors_PWM_Config();
  
  Calculate_Colors_Shifting_and_Sampling();
  
  ReloadPWMColors();//���������� � �������� ��� ������
  SetRGB(0,0,0);
  //------------SETUP DAC--------------------
  X=centreX;Y=centreY;
  DAC_XY_Config();//��� ���������
  _setDACs((uint16_t)centreX,(uint16_t)centreY);//���������� �������� �������� DAC X,Y � ����� (���������� ��������)
  //-------------SETUP PLAY_TIMERS--------------
  PlayTimers_Config();  
  //---------------------------------
#ifndef DISCOVERY_BOARD
  SDCard_Init();//hardware
#endif  
  
  button[8]=1;
  //---------------------------------
}
//----------------------------------------------------------------------------//
//���������� ��������� ��������� ����� 32 ��� 0-2^32 = 4294967295 (0xFFFFFFFF)
uint32_t u32Rnd(){
while((RNG->SR & RNG_FLAG_DRDY) == (uint8_t)RESET);  // ��� ��������� ������ ����������
return RNG->DR;
}
//float[0-1.0f)
float fRnd(){return (float)u32Rnd()/4294967296.0f;}
//int random �� min �� max ������������
int iRnd(int min,int max)
{
  float v=(fRnd()+0.000001f)*(float)(max-min);
  return (int)v;
}
//----------------------------------------------------------------------------//
/*
void RNG_config()
{
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,ENABLE);
	RNG_Cmd(ENABLE);
}

uint32_t GetRNG()
{
 while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET);  //Wait for a random number ready.
 uint32_t data = RNG_GetRandomNumber();   //Reading
 return data;
}
*/
//-------------------------������������� ADC1,2------------------------------------------------
//�������������� �������������� 4 �������������� ������� � ��������������� ������������������
//(���������� �������.)
/*
void ADC_Inj_Config(void)
{
  ADC_DeInit();//Deinitializes all ADCs peripherals registers to their default reset values. 
  // Enable ADC1,2 and GPIO clocks **************************************** /
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2, ENABLE);//ADC1;ADC2 //84 mhz
	
  ADC_InitTypeDef       ADC1_InitStructure;
  ADC_InitTypeDef       ADC2_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_A_InitStructure;	//x1 x2
  GPIO_InitTypeDef      GPIO_C_InitStructure; //y1 y2
    
  //�������� ������������� ������ ����������� ������� �� PA1,�������-VCC � GND
	
  // Configure ADC1 pins as analog input ****************************** /
  GPIO_A_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2 ;//PA1,PA2
  GPIO_A_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//analog input
  GPIO_A_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//no pull-up,no pull-down
  GPIO_Init(GPIOA, &GPIO_A_InitStructure);
  
  // Configure ADC2 pins as analog input ****************************** /
  GPIO_C_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2 ;//PC1,PC2
  GPIO_C_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_C_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_C_InitStructure);
  
  // ADC1 Init *************MASTER****************************************** /
  ADC_StructInit(&ADC1_InitStructure);//���������� ��������� ����� ����� ������������� ���������
  ADC1_InitStructure.ADC_Resolution = ADC_Resolution_12b;//��������=2*12 ���(X1+X2)
  ADC1_InitStructure.ADC_ScanConvMode = ENABLE;//=��� ��������� ��������� ������� � �������
  ADC1_InitStructure.ADC_ContinuousConvMode =DISABLE;// DISABLE;// ��� ��������������� ����� ������ ���� �������(2)
  ADC1_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no source
  ADC1_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC1_InitStructure.ADC_NbrOfConversion = 0;
  ADC_Init(ADC1, &ADC1_InitStructure);
  // ADC2 Init ************SLAVE******************************************** /
  ADC_StructInit(&ADC2_InitStructure);//���������� ��������� ����� ����� ������������� ���������
  ADC2_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC2_InitStructure.ADC_ScanConvMode = ENABLE;//=��� ��������� ��������� ������� � �������
  ADC2_InitStructure.ADC_ContinuousConvMode =DISABLE;//DISABLE;// ��� ��������������� ����� ������ ���� �������(2)
  ADC2_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC2_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC2_InitStructure.ADC_NbrOfConversion = 0;
  ADC_Init(ADC2, &ADC2_InitStructure);
  // ADC common Init ********************************************************** /
  ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_InjecSimult;//��� ��� ������������(injected dual mode)
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//84 ���/2=42��� ADCCLK
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//��� DMA
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//delay
  ADC_CommonInit(&ADC_CommonInitStructure);
  //����� ����� = 42 ���/(28*2+5)=688524.59016393442622950819672131 ���� ��������� XY ADC
  ADC_InjectedSequencerLengthConfig(ADC1, 2);//����� ����������
  ADC_InjectedSequencerLengthConfig(ADC2, 2);
  //������������� �������������� PA1 PA2 � PC1 PC2
  //freq=ADCCLK/ADC_SampleTime = 1.5 MHZ
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_28Cycles);//PA1
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_28Cycles);//PC1
	
  ADC_InjectedChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_28Cycles);//PC2
  ADC_InjectedChannelConfig(ADC2, ADC_Channel_2, 2, ADC_SampleTime_28Cycles);//PA2
  
  ADC_SetInjectedOffset(ADC1, ADC_InjectedChannel_1,0);//The channel 1 ADC1 will be digitized in time ADC2 channel 2
  ADC_SetInjectedOffset(ADC1, ADC_InjectedChannel_2,0);//The channel 2 ADC1 will be digitized in time ADC2 channel 1
  
  ADC_SetInjectedOffset(ADC2, ADC_InjectedChannel_1,0);
  ADC_SetInjectedOffset(ADC2, ADC_InjectedChannel_2,0);

//	����������� (Discontinuous)
//	����� �������������� �� ��� ������ �� ���,
//	� ���� �� ��������� ������� ������������� �����. � ��������� ��� ��� ����������� �������,
//	������������ ��������������, ����� �������������� ��������� ������ ������� � �.�.
	
  ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_None);//no source
  ADC_AutoInjectedConvCmd(ADC1, DISABLE);//?????��������� ���������� ��������������� ������.����� �����.
  ADC_InjectedDiscModeCmd(ADC1, DISABLE);
  
  ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_None);//no source
  ADC_AutoInjectedConvCmd(ADC2, DISABLE);//?????��������� ���������� ��������������� ������.����� �����.
  ADC_InjectedDiscModeCmd(ADC2, DISABLE);
  
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);//interrrupt for dual mode 
 
  NVIC_SetPriority(ADC_IRQn, 0);
  NVIC_EnableIRQ(ADC_IRQn);
  // Enable ADCx 
  ADC_Cmd(ADC1, ENABLE);//on adc
  ADC_Cmd(ADC2, ENABLE);
  //stm32f4xx.h [304]-ADC registers  [1244] -Bit definitions
  ADC_SoftwareStartInjectedConv(ADC1); //for ADC-testing  
}
*/
//----��������� ���������� ������� ADC �������������� ��������---------------
void ADC_Reg_Config(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ADC1 84 mhz
  
  ADC_DeInit();//Deinitializes all ADCs peripherals registers to their default reset values. 
	
  ADC_InitTypeDef       ADC_struct;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_structure; //PA7=sys temp
  // Configure ADC1 pins as analog input ****************************** /
  //PA7=7 ch ADC1 STEMP
  //PA6=6 ch ADC1 GTEMP
  GPIO_StructInit(&GPIO_structure);
  GPIO_structure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_structure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_structure.GPIO_PuPd = GPIO_PuPd_UP ;//�������� �����
  GPIO_structure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_structure);
  // ADC1 Init 
  ADC_StructInit(&ADC_struct);//���������� ��������� ����� ����� ������������� ���������
  ADC_struct.ADC_Resolution = ADC_Resolution_12b;//��������=2*12 ���(X1+X2)
  ADC_struct.ADC_ScanConvMode = ENABLE;//��� ��������� ��������� ������� � �������
  ADC_struct.ADC_ContinuousConvMode =DISABLE;// ��� ��������������� ����� ������ ���� �������(2)
  ADC_struct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no source
  ADC_struct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_struct.ADC_NbrOfConversion = 2;
  ADC_Init(ADC1, &ADC_struct);
  // ADC common Init
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;//84 ���/8=10.5 ��� ADCCLK
  ADC_CommonInitStructure.ADC_DMAAccessMode =  ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//delay
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  ADC_RegularChannelConfig(ADC1,ADC_Channel_6, 1, ADC_SampleTime_84Cycles);//STEMP rank 1
  ADC_RegularChannelConfig(ADC1,ADC_Channel_7, 2, ADC_SampleTime_84Cycles);//GTEMP rank 2
  //----------DMA----------------------
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedValue[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);

  DMA_Cmd(DMA2_Stream0, ENABLE);
  //-------------IT---------------------
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
   
  /*NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);*/
  
  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  
  //NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn;
  //NVIC_Init(&NVIC_InitStruct);

  //---------------------------------------------
  //stm32f4xx.h [304]-ADC registers  [1244] -Bit definitions
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConv(ADC1);
  //NVIC_EnableIRQ (DMA2_Stream0_IRQn);
}
//-----ADC DMA stream--------------
//void DMA2_Stream0_IRQHandler(void)
//�������� ���������� ������ ��������������
#define __atxMinPWM     50
#define __lasersMinPWM  40
#define __atxDiodeHighLevel 640
#define __lasersDiodeHighLevel 640
#define __atxDiodeLowLevel 500
#define __lasersDiodeLowLevel 500

void ADC_check_complete()
{
  if(DMA_GetFlagStatus( DMA2_Stream0,DMA_FLAG_TCIF0)!=SET)return;
  DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0|DMA_FLAG_FEIF0|DMA_FLAG_TEIF0);
  ADC_ClearITPendingBit(ADC1,ADC_IT_EOC|ADC_IT_OVR);
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, DISABLE);
     
  static int lasers_PWM=__lasersMinPWM; // %
  static int atx_PWM=__atxMinPWM;       // % 
   
   uint16_t ATXtempCurrentLevel=ADC1ConvertedValue[0];
   uint16_t Stemp=ADC1ConvertedValue[1];
   
   //temperature ATX
   if(ATXtempCurrentLevel>__atxDiodeHighLevel)ATXtempCurrentLevel=__atxDiodeHighLevel;
   int nPWM=(__atxDiodeHighLevel-ATXtempCurrentLevel)*100/(__atxDiodeHighLevel-__atxDiodeLowLevel)+__atxMinPWM;//% *2000 of PWM duty
   atx_PWM=(atx_PWM*30 + nPWM*70)/100;
   int vpwm=atx_PWM*20;
   if(vpwm>2000)vpwm=2000;
   TIM3->CCR3=(uint16_t)vpwm;//PB0 TIM3_CH3 GFAN PWM (0-2000)
   
   //temperature STEMP lasers
   //Stemp=Stemp>=lasersDiodeLevel?lasersDiodeLevel:(lasersDiodeLevel-Stemp+30)*20;//% *2000 of PWM duty
   if(Stemp>__lasersDiodeHighLevel)Stemp=__lasersDiodeHighLevel;
   nPWM=(__lasersDiodeHighLevel-Stemp)*100/(__lasersDiodeHighLevel-__lasersDiodeLowLevel)+__lasersMinPWM;//% *2000 of PWM duty
   lasers_PWM=(lasers_PWM*30 + nPWM*70)/100;
   vpwm=lasers_PWM*20;
   if(vpwm>2000)vpwm=2000;
   TIM3->CCR4=(uint16_t)vpwm;//PB1 TIM3_CH4 SFAN PWM (0-2000)
   //to next iteration
   DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
   ADC_SoftwareStartConv(ADC1); //new start
}
//---------------��������� ���� ���������� ��������-------------------------
void Power_pin_config()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //�����-�������������� �������
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP ;//���-�����������
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//������������ �����
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void Power_ON()
{
  GPIO_SetBits(GPIOA, GPIO_Pin_10);
  Power=true;
}

void Power_OFF()
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_10);
  Power=false;
  if(WndGrad!=CLOSE_ANGLE)Wnd_CLOSE();
}

void Power_Toggle()
{
  if(Power)Power_OFF();else Power_ON();
}

//���������� ��� �����������
void Power_off_timer_func()
{
  if(Power)Power_OFF();
  if(ICmode==IM_SD)
  {
    ICmode=IM_WLAN;
  }
}
//---------------��������� ���-------------------------
void ServoPinConfig(bool on)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);  
  
  GPIO_InitStructure.GPIO_Mode = on?GPIO_Mode_AF:GPIO_Mode_IN; //�����-�������������� �������
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP ;//���-� ��������� �����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//�������� �����
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//������������ �����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PC6
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  if(on)GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);//PWM
}

void FANs_Servo_PWM_config()
{
  //��� ���������� ����� ������� 50�� (������=20��)
  //20000 mks/2000=10 mks CLK
  // 1 ms=left=100 CLK; 1.5ms=middle=150 CLK; 2ms=right=200 CLK, period=20ms
  
  GPIO_InitTypeDef GPIO_InitStructure;
  //---config SFAN PWM PIN PB1,TIM3_CH4
  //---config GFAN PWM PIN PB0,TIM3_CH3
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;//����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //�����-�������������� �������
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP ;//���-� ��������� �����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//�������� �����
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//������������ �����
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);//GFAN PWM
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);//SFAN PWM
  //---config servo PIN PC6,TIM3_CH1
  //GPIO_StructInit(&GPIO_InitStructure);  
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PC6
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //�����-�������������� �������
  //GPIO_InitStructure.GPIO_OType= GPIO_OType_PP ;//���-� ��������� �����
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//�������� �����
  //GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//������������ �����
  //GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);//PWM
  
  //-----�������� ������� ���-------
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //84 MHz
  /* Timers configuration in PWM mode */
  TIM_DeInit(TIM3);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  //TIM3 84000000/840/2000=50Hz
  TIM_TimeBaseStruct.TIM_Prescaler=839; //CLK/x TIM3
  TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Down;  
  TIM_TimeBaseStruct.TIM_Period=(uint16_t)2000;//0x0000 - 0xFFFF ������ ��� ,50 ��
  TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);
  TIM_UpdateRequestConfig  (TIM3, TIM_UpdateSource_Regular);
  //PWM
  TIM_OCInitTypeDef oc_init;
  TIM_OCStructInit(&oc_init);
  oc_init.TIM_OCMode = TIM_OCMode_PWM1;//�����1 front align(TIM_OCMode_PWM2-�� ������)
  oc_init.TIM_OutputState = TIM_OutputState_Enable;//���������� � ������
  oc_init.TIM_OCPolarity = TIM_OCPolarity_High;  //  ����������
  //SFAN= 4 ch TIM3=PB1
  oc_init.TIM_Pulse = 0; //��������� ����������
  TIM_OC4Init(TIM3,&oc_init);  
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  //GFAN= 3 ch TIM3=PB0
  //oc_init.TIM_Pulse = 0; //��������� ����������
  TIM_OC3Init(TIM3,&oc_init);  
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
   
  //FRONT servo= 1 ch TIM3=PC6
  //oc_init.TIM_Pulse = 100; //��������� ����������
  TIM_OC1Init(TIM3,&oc_init);  
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
 // TIM_CtrlPWMOutputs(TIM1,ENABLE);//������ ��� TIM1,TIM8
  /* Disable the Interrupt sources */
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM3,ENABLE);   // ��������� ���� ���
  
  //TIM3->CCR1=245;
  //TIM3->CCR1=47;
  //TIM3->CCR1=ServoGradToPWM(180,&wndServo);
  //while(1);
}
//----------------------------------------------------------------------------//
//������������� ���� � �������� ���
uint16_t ServoGradToPWM(uint16_t grad,SERVO_PAR* servoLimits)
{
  float oneGrad_clks=(float)(servoLimits->max-servoLimits->min)/(float)servoLimits->angle;
  float val=(float)grad*oneGrad_clks+(float)servoLimits->min;
  uint16_t uv=(uint16_t)val;
  if(uv>servoLimits->max)uv=servoLimits->max;
  else if(val<servoLimits->min)uv=servoLimits->min;
  return uv;
}

void _wnd_servo_angle(uint16_t angle)
{
  ServoPinConfig(true);
  TIM3->CCR1=ServoGradToPWM(WndGrad=angle,&wndServo);
  ServoOffPwrCycles=SERVO_OFF_CNTR;
}

void Wnd_OPEN()
{
  _wnd_servo_angle(OPEN_ANGLE);
}

void Wnd_CLOSE()
{
  _wnd_servo_angle(CLOSE_ANGLE);
}
//---------------���������� ADC----------------------------------------------
void ADC_IRQHandler(void)//�������������� ADC ���������
{
  ;
}

//----------------------------------------------------------------------------//
void SetRGB(uint8_t r,uint8_t g,uint8_t b)
{
  _setR(r);//R channel 1 PWM
  _setG(g);//G channel 2 PWM
  _setB(b);//B channel 1 PWM
}
//----------------------------------------------------------------------------//
void DAC_XY_Config()//��� ���������,2*12 ��� ������
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = DAC_X_Pin|DAC_Y_Pin;//PA4=X out,PA5=Y out
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  
  DAC_InitTypeDef DAC_InitStructure;
  DAC_StructInit(&DAC_InitStructure);
  DAC_Init(DAC_Channel_1, &DAC_InitStructure) ; 
  DAC_Init(DAC_Channel_2, &DAC_InitStructure) ; 
  //DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState):
  DAC->CR |= DAC_CR_EN1;
  DAC->CR |=DAC_CR_EN2;
}
//----------------------------------------------------------------------------//
void PID_Timer_Config(uint16_t TimeStep)//������ ������� �������������� ADC
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  //PCLK1=42 MHZ
  TIM_TimeBaseStruct.TIM_Prescaler=0;
  TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Down;
  TIM_TimeBaseStruct.TIM_Period=TimeStep;
  TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStruct);
  TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  NVIC_EnableIRQ(TIM2_IRQn); 
  
  TIM_Cmd(TIM2,ENABLE);
}
//----------------------------------------------------------------------------//

//��������� �������
void Timer_Config(uint8_t APB_1_or_2,uint32_t RCC_APB_Periph,
                  TIM_TypeDef* TIMx,
                  uint16_t prescaler,uint16_t period,
                  uint16_t TIM_CounterMode,
                  uint8_t IRQ_Type,//255=��� ����������
                  bool StartNow)//������?
{
  //-----�������� ��������-------
  if(APB_1_or_2==1)RCC_APB1PeriphClockCmd(RCC_APB_Periph, ENABLE);
  else if(APB_1_or_2==2)RCC_APB2PeriphClockCmd(RCC_APB_Periph, ENABLE); 
  else HardFault_Handler();//������������ ����
    
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  //PCLK2=84 MHZ,������������ ������� 168 ���.
  //PCLK1=42 MHZ,������������ ������� 84 ���.
  TIM_TimeBaseStruct.TIM_Prescaler=prescaler;
  TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode;//(���� �� ���� �����)
  /*
  ��� ����� �� ���������� (���������) � ������� ������� ���� ��������� ����� ����������� ��������� �������:
  TIMx->CNT = msec-1;//��������� ����� ����������� � ������� �������(������������ ������ 1��)
  � ��� ����� �� ���������� (���������) ���:
  TIMx->CNT = 65535-(msec-1);//��������� ����� ����������� � ������� �������
  */
  TIM_TimeBaseStruct.TIM_Period=period;//PCLK2:4000*42000=168000000=1sec
  TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIMx,&TIM_TimeBaseStruct);
  
  //TIMx->CR1 &= (uint16_t)~TIM_CR1_CEN;
  TIMx->SR &= (uint16_t)~TIM_SR_UIF; //���������� ���� UIF(� �������� �������)
  TIMx->DIER &= (uint16_t)~TIM_DIER_UIE;//��������� ���������� �� ������������
  
  if(IRQ_Type!=255)NVIC_EnableIRQ((IRQn_Type)IRQ_Type);//core_cm4.h 1071 
  if(StartNow==true)TIMx->CR1 |= TIM_CR1_CEN;//������,��������� � �����������
}
//----------------------------------------------------------------------------//
//�������� �������� � ������ �������
void TIMxOnePulseAfterPeriod(TIM_TypeDef* TIMx,uint16_t prescaler,uint16_t reload)
{
  TIMxLoadTimerAndStart(TIMx,prescaler,reload,TIM_CR1_OPM);
}

//������� ����� ����� ������������ ��� Ftclk / (TIMx_PSC + 1)
//�������� �������� � ������ �������,� ������ ������
//TIM_CR1_URS=���������� �����,������ ��������� ����
//TIM_CR1_OPM=����� ���������� ��������,������ ��������� ���� �������������

void TIMxLoadTimerAndStart(TIM_TypeDef* TIMx,uint16_t prescaler,uint16_t reload,uint16_t pulse_mode)
{
  TIMx->PSC = (uint16_t)(prescaler-1);// Set the Prescaler value 
  TIMx->ARR = (uint16_t)(reload);//������� �������� �� ���������� ��������
  TIMx->CNT =0;//���� �����
  TIMx->EGR = TIM_EGR_UG; //TIM event generation register
  TIMx->SR &= (uint16_t)~TIM_SR_UIF;// Clear the IT pending Bit
  TIMx->CR1 = pulse_mode|TIM_CR1_CEN;//�����+������,��������� � �����������
  TIMx->DIER = TIM_DIER_UIE; //��������� ���������� ��� ������������ ��������
}
//��������� � ���������� ���������� �������
void TIMxStopAndDisableIT(TIM_TypeDef* TIMx)
{
 /* Disable the TIM Counter */
 TIMx->CR1 &= (uint16_t)~TIM_CR1_CEN;
 TIMx->DIER &= (uint16_t)~TIM_DIER_UIE;//��������� ����������
 TIMx->SR = (uint16_t)~TIM_SR_UIF;// Clear the IT pending Bit
}
//----------------------------------------------------------------------------//
//���������� �������� � ������ ��� ������� �� ����������� �������� � ������ CLK �������
//AHB2=max 168 MHz AHB1=max 84 MHz
void getTIM_par(uint16_t* pprescaler,uint16_t* pperiod,uint32_t delay)
{
  *pprescaler=(uint16_t)(delay/(uint32_t)65536)+1;
  *pperiod=(uint16_t)(delay-((uint32_t)(*pprescaler))*(uint32_t)65536);
}
//----------------------------------------------------------------------------//
void LScan_Denit(){
  _setRGB(0,0,0);//��������� ������
}
//----------------------------------------------------------------------------//
void ReloadPWMColors()//����������� ��� ������ � ��������� ������
{
   //int pwm_min=max(max(R_pwm_min,G_pwm_min),B_pwm_min);
   //int pwm_max=max(max(R_pwm_max,G_pwm_max),B_pwm_max);
   //int pwm_tresh=max(max(R_Tresh,G_Tresh),B_Tresh);
   //float clrPwmPrcWeight=(float)COLORS_PWM_PERIOD/100.0f;
   TIM1->PSC =(uint16_t)(168/Colors_PWM_Freq-1);//��������
   TIM1->ARR=(uint16_t)(COLORS_PWM_PERIOD);
     //+(float)(pwm_min+pwm_tresh)*clrPwmPrcWeight);//�������� ������������: PWM+�������
   
   Calculate_PWM_Table(RED_LASER);
   Calculate_PWM_Table(GREEN_LASER);
   Calculate_PWM_Table(BLUE_LASER);
}
//----------------------------------------------------------------------------//
void Calculate_PWM_Table(int color)
{
  /*
  ������������ �������� ������� �������������� ������� �������� ��������� ������� (����� ���������)
  */
  int pwmmin=(color==RED_LASER?R_pwm_min:(color==GREEN_LASER?G_pwm_min:B_pwm_min));//������� ���
  int pwmmax=(color==RED_LASER?R_pwm_max:(color==GREEN_LASER?G_pwm_max:B_pwm_max));//�������� ���
  int power=(color==RED_LASER?R_power:(color==GREEN_LASER?G_power:B_power));
  int tresh=(color==RED_LASER?R_Tresh:(color==GREEN_LASER?G_Tresh:B_Tresh));
  float clrPwmPrcWeight=(float)COLORS_PWM_PERIOD/100.0f;
  uint16_t pwmReg=(uint16_t)( (float)(100-pwmmin) * clrPwmPrcWeight);//��������� ����� ��� ��� �����
  float value;
  
  for (int n=0;n<=255;n++)
  {
    value= (float)pwmmin*clrPwmPrcWeight;//����������� ���������� ���
    
    if(n>0)
    {
      value+=tresh*clrPwmPrcWeight+(float)pwmmax/100.0f*(float)power/100.0f*
          (float)n/255.0f *(float)pwmReg;//������� �����
    }
    
    if(value>(float)COLORS_PWM_PERIOD)value=(float)COLORS_PWM_PERIOD;//TTL modulation
    //(uint16_t)((float)(n)/255.0f*(float)COLORS_PWM_PERIOD);//
    if(color==RED_LASER)R_PWM[n]=(uint16_t)value;
    else if(color==GREEN_LASER)G_PWM[n]=(uint16_t)value;
    else B_PWM[n]=(uint16_t)value;
  }

}
//----------------------------------------------------------------------------//
void Calculate_Colors_Shifting_and_Sampling()//������ ������ ��������
{
  uint16_t maxPeriod=(uint16_t)42000;//������������ ����� ������ � ����� ������
  uint32_t don=(uint32_t)(RGB_on_shift/1000);
  uint32_t doff=(uint32_t)(RGB_off_shift/1000);
  //������� ����������
  //f(n)=lg(((n-beg_level)*Linearity)+1.0001 )/lg((up_limit-beg_level)*Linearity+1.0001);
  
  for (int n=1;n<256;n++)//��� ���� ������
  {
    float smplPeriod=maxPeriod/n-100.0f;
    //��������������� ������� ���������� ������ [1.01;100] (1.01-�������� �������),����� ������ ����� ����������� ������� [0;1]
    //�������� �� ��������� ������������� (0;1] 
    float k_shift_on=n>=don?log10(((float)n-(float)don)*Linearity+1.0001f)/log10(((float)MAX_DISCR_MS-(float)don)*Linearity+1.0001f):0;
    float k_shift_off=n>=doff?log10(((float)n-(float)doff)*Linearity+1.0001f)/log10(((float)MAX_DISCR_MS-(float)doff)*Linearity+1.0001f):0;
    //���������� �������� ������� ������ � ������ �������� �������    
    offsetRGBon[n]=(uint16_t)(k_shift_on*RGB_on_k*smplPeriod);
    offsetRGBoff[n]=(uint16_t)(k_shift_off*RGB_off_k*smplPeriod);
   /* if(n*1000<=DPSS_DISCR)ddpss[n]=0;
      else 
      {
        ddpss[n]=(uint8_t)(((int)n*1000-DPSS_DISCR)*100/(MAX_DISCR-DPSS_DISCR))/2;//� ���������
        if(ddpss[n]>(uint8_t)100)ddpss[n]=(uint8_t)100;
      }*/
    
    if(offsetRGBon[n]<1)offsetRGBon[n]=1;
    if(offsetRGBoff[n]<1)offsetRGBoff[n]=1;
  }
  //ddpss[0]=ddpss[1];
  //smplPeriod[0]=maxPeriod;
  offsetRGBon[0]=offsetRGBon[1];offsetRGBoff[0]=offsetRGBoff[1];  
}
//----------------------------------------------------------------------------//
uint32_t DPM_mic_clk=0;
void MEMS_DPM_MIC_Config(void)
{
  //������� SPI_CS MEMS � HIGH �� DISCOVERY (PE3) �� ��������� ��������� ���������
  GPIO_InitTypeDef GPIO_InitStructure;
  //---config PIN, PE9,TIM1_CH1
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//CLK
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP ;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);//R
  //-----�������� ������� ���-------
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //168 MHz
  /* Timers configuration in PWM mode */
  TIM_DeInit(TIM1);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  //2 MHZ
  TIM_TimeBaseStruct.TIM_Prescaler=(uint16_t)(0); //CLK/x ��� ��������� .CLK=168 ��� ��� TIM1
  TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;  //������� �����  
  TIM_TimeBaseStruct.TIM_Period=(uint16_t)84;//0x0000 - 0xFFFF ������ ���
  TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
  TIM_UpdateRequestConfig  (TIM1, TIM_UpdateSource_Regular);
  //PWM
  TIM_OCInitTypeDef oc_init;
  TIM_OCStructInit(&oc_init);
  oc_init.TIM_OCMode = TIM_OCMode_PWM1;//�����1 front align(TIM_OCMode_PWM2-�� ������)
  oc_init.TIM_OutputState = TIM_OutputState_Enable;//���������� � ������
  oc_init.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;  // ������������� ����������(3V=�������)
  //CLK= 1 ch TIM1=PE9
  oc_init.TIM_Pulse = 42; //��������� ����������
  TIM_OC1Init(TIM1,&oc_init);  
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
   
  TIM_CtrlPWMOutputs(TIM1,ENABLE);//������ ��� TIM1,TIM8
  /* Disable the Interrupt sources */
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  //-------------------------------------------
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource0);
  
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    
  EXTI_InitTypeDef CLK_mic;
  EXTI_StructInit(&CLK_mic);
  //EXTI0_IRQn
  CLK_mic.EXTI_Line = EXTI_Line0;//PB0
  CLK_mic.EXTI_Mode = EXTI_Mode_Interrupt;    
  // ����������� � ����������� �� ��������� ������ (�����,����)
  CLK_mic.EXTI_Trigger = EXTI_Trigger_Rising; 
  CLK_mic.EXTI_LineCmd = ENABLE;    // ���
  EXTI_Init(&CLK_mic);
  
  EXTI_ClearITPendingBit(EXTI_Line0);
  NVIC_EnableIRQ(EXTI0_IRQn); 
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); 
  //-------------------------------------------
  DAC_XY_Config();
   //-------------------------------------------  
  TIM_Cmd(TIM1,ENABLE);   // ��������� ���� ���
 }

void EXTI0_IRQHandler()
{
  EXTI_ClearITPendingBit(EXTI_Line0);
  DPM_mic_clk++;
  LSCAN2_LEDToggle(LEDBLUE); 
}
//----------------------------------------------------------------------------//
void Colors_PWM_Config(void)
{
  //������� SPI_CS MEMS � HIGH �� DISCOVERY (PE3) �� ��������� ��������� ���������
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_SetBits(GPIOE, GPIO_Pin_3);//cs=low,high=off chip SPI
  //---config R color PIN, PE9,TIM1_CH1
  //---config G color PIN, PE11,TIM1_CH2
  //---config B color PIN, PE13,TIM1_CH3
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13;//R|G|B
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP ;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);//R
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);//G
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);//B
  //-----�������� ������� ���-------
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //168 MHz
  /* Timers configuration in PWM mode */
  TIM_DeInit(TIM1);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  //��������� ��������� �� ������ ���,�������� TIM1 +1=CLK/2=84MHz
  //TIM1=RGB
  TIM_TimeBaseStruct.TIM_Prescaler=(uint16_t)(168/Colors_PWM_Freq-1); //CLK/x ��� ��������� .CLK=168 ��� ��� TIM1
  TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;  //������� �����  
  TIM_TimeBaseStruct.TIM_Period=(uint16_t)COLORS_PWM_PERIOD;//0x0000 - 0xFFFF ������ ���
  TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
  TIM_UpdateRequestConfig  (TIM1, TIM_UpdateSource_Regular);
  //PWM
  TIM_OCInitTypeDef oc_init;
  TIM_OCStructInit(&oc_init);
  oc_init.TIM_OCMode = TIM_OCMode_PWM1;//�����1 front align(TIM_OCMode_PWM2-�� ������)
  oc_init.TIM_OutputState = TIM_OutputState_Enable;//���������� � ������
  oc_init.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;  // ������������� ����������(3V=�������)
  //R= 1 ch TIM1=PE9
  oc_init.TIM_Pulse = 0; //��������� ����������
  TIM_OC1Init(TIM1,&oc_init);  
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  //G=2 ch TIM1=PE11
  oc_init.TIM_Pulse = 0; 
  TIM_OC2Init(TIM1,&oc_init);  
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  //B=3 ch TIM1=PE13
  oc_init.TIM_Pulse = 0; 
  TIM_OC3Init(TIM1,&oc_init);  
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  
  TIM_CtrlPWMOutputs(TIM1,ENABLE);//������ ��� TIM1,TIM8
  /* Disable the Interrupt sources */
  TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM1,ENABLE);   // ��������� ���� ���
  
 }
//----------------------------------------------------------------------------//
void TIM1_BRK_TIM9_IRQHandler() //������ �������� ������ R
{
  TIM9->DIER &= (uint16_t)~TIM_DIER_UIE;//��������� ����������
  TIM9->SR = (uint16_t)~TIM_SR_UIF;// Clear the IT pending Bit
  TIM1->CCR1=R_PWM[R=newR_pwm];//R channel 1 PWM
  //if(G!=newG_pwm)TIM1->CCR2=G_PWM[G=newG_pwm];//G channel 2 PWM
  TIM1->CCR3=B_PWM[B=newB_pwm];//B channel 3 PWM
}
//----------------------------------------------------------------------------//
void TIM1_UP_TIM10_IRQHandler() //������ �������� ������ G
{
  /*TIM1->SR = (uint16_t)~TIM_SR_UIF;// Clear the IT pending Bit
  static uint32_t cycles=0;
  cycles++;
  uint32_t len=2000;
  if(cycles==len)
  {
    float mclk=(float)DPM_mic_clk/(float)len * 4095.0f;
    uint16_t vo=(uint16_t)mclk;
    DAC->DHR12R1=vo;//PA4
    DPM_mic_clk=0;
    cycles=0;
  }
  
  return;*/
  
  TIM10->DIER &= (uint16_t)~TIM_DIER_UIE;//��������� ����������
  TIM10->SR = (uint16_t)~TIM_SR_UIF;// Clear the IT pending Bit
  if(G!=newG_pwm)TIM1->CCR2=G_PWM[G=newG_pwm];//G channel 2 PWM
}
//----------------------------------------------------------------------------//
void TIM1_TRG_COM_TIM11_IRQHandler() //������ ���-��������� B
{
  TIM11->DIER &= (uint16_t)~TIM_DIER_UIE;//��������� ����������
  TIM11->SR = (uint16_t)~TIM_SR_UIF;// Clear the IT pending Bit
  if(B!=newB_pwm)TIM1->CCR3=B_PWM[B=newB_pwm];//B channel 3 PWM
}
//----------------------------------------------------------------------------// 
bool SD_detect_processing(void)
{
  uint8_t det=SD_Detect();Sleep(100);
  while(det!=SD_Detect()){Sleep(100);det=SD_Detect();} 
  if(det==SD_PRESENT)//SD inserted
  {
    if(SDCardMounted==0)
    {
      oldICmode=ICmode;
      ICmode=IM_SD;
      SDCardMounted=1;
      return true;
    }
  }
  else //SD deleted
  {
    if(!det && SDCardMounted)
    {
      //------------------------
      f_mount(NULL,_T("1:/"),0);//unmount
      SDCardMounted=0;
      SDIO_ClockCmd(DISABLE);
      if(ICmode==IM_SD)
      {
        ICmode=oldICmode;
      }
      return true;
    }
  }
  return false;
}
//------------------------------------------------------------------------------
//���������������� SD DETECT ��� ������� �� �����/���� �������� ���� Detect
/*void SD_CARD_detect_reconfig(EXTITrigger_TypeDef rising_or_falling)
{
  NVIC_DisableIRQ(SD_DETECT_EXTI_IRQChannel); 
  
  EXTI_InitTypeDef exti_detect;
  EXTI_StructInit(&exti_detect);
  exti_detect.EXTI_Line = SD_DETECT_EXTI_LINE;
  exti_detect.EXTI_Mode = SD_DETECT_EXTI_MODE;    
  // ����������� � ����������� �� ��������� ������ (�����,����)
  exti_detect.EXTI_Trigger = rising_or_falling; 
  exti_detect.EXTI_LineCmd = ENABLE;    // ���
  EXTI_Init(&exti_detect);
  
  EXTI_ClearITPendingBit(SD_DETECT_EXTI_LINE);
  NVIC_EnableIRQ(SD_DETECT_EXTI_IRQChannel); 
}*/
//------------------------------------------------------------------------------
//������������ SD
/*int SD_CARD_processing(void* ctx)
{
  IS_RES rs=IS_OK;
    
  if(!SDCardMounted)
  {
    SDIO_ClockCmd(ENABLE);
    Sleep((uint32_t)300);
    FRESULT rs=f_mount(FS_SDIO_disk,_T("1:/"),0);
    if(rs==FR_OK)
    {
      SDCardMounted=1;
    }
    else 
    {
      goto destroy_play_task;
    }
  }
  
  //PLAY +++++++++++++++
  if(Power_timer)DeleteSystemTimer(&Power_timer);
  if(!Power)Power_ON();
  if(WndGrad>0)Wnd_OPEN();
  
  rs=Proccess_Player();//������ ������
  
  //�� ������ ��� ������ ��� ������������ �� �������?
  if((rs & IS_ERR)!=IS_ERR)return NULL;
  
  //----���������� ������ � �������������--------
destroy_play_task:
  SetRGB(0,0,0);
  ICmode=oldICmode;
   
  //������������� SD
  //-----------------if(SD_CARD_task)DeleteSystemTask(&SD_CARD_task);
  //-----------------SD_CARD_task=NULL;  
  f_mount(NULL,_T("1:/"),0);//unmount
    
  SDIO_ClockCmd(DISABLE);
    
  return 0;
}*/
//----------------------------------------------------------------------------//
//=================================================================================================
void LED_PanicLoop(uint32_t numLoops) {
  uint32_t i=0;
  uint32_t l=numLoops; //1->0=return or 0=infinity
  for(;;){
    LSCAN2_LEDOn(LEDBLUE);LSCAN2_LEDOn(LEDRED);//OB
    LSCAN2_LEDOff(LEDWHITE);LSCAN2_LEDOff( LEDGREEN);//GR
    i=5000000;
    while(i--);
    LSCAN2_LEDOff(LEDBLUE);LSCAN2_LEDOff(LEDRED);//OB
    LSCAN2_LEDOn(LEDWHITE);LSCAN2_LEDOn( LEDGREEN);//GR
    i=5000000;
    while(i--);
    if(l>0)
    {
      if(l==1)
      {
        l=1;
        return;
      }
      else l--;
    }
  }
}
//----------------------------------------------------------------------------//
void PlayTimers_Config()//������ ������������ .������==1 ��
{
  NVIC_InitTypeDef NVIC_InitStructure;
  //PCLK2=84 MHZ,������������ ������� 168 ���.
  //PCLK1=42 MHZ,������������ ������� 84 ���.
  //������ ������������,�������� ��������� � ���������� 
  //--------------------------------------------
  //24 ������ ������ ��������� �����,��������� ���������� ����������, ����� ������� ��������� 0 (core_cm4.h)
  RCC_ClocksTypeDef RCC_Sys;
  RCC_GetClocksFreq(&RCC_Sys);
  
  SysTick->CTRL  =  0;//reset
  
  
  //---------SETUP COLORS SHIFTING--------------
  //������� ������ ������ TIM9=rb,10=g
  
  Timer_Config(2,RCC_APB2Periph_TIM9,
               TIM9,
               4000,42000,
               TIM_CounterMode_Up,
               TIM1_BRK_TIM9_IRQn,
               false);//��������� ������� R-color shift
  /*Timer_Config(2,RCC_APB2Periph_TIM10,
               TIM10,
               4000,42000,
               TIM_CounterMode_Up,
               TIM1_UP_TIM10_IRQn,
               false);//��������� ������� G-color shift
  */
  /*Timer_Config(2,RCC_APB2Periph_TIM11,
               TIM11,
               4000,42000,
               TIM_CounterMode_Up,
               TIM1_TRG_COM_TIM11_IRQn,
               false);//��������� ������� B-color shift
  */
  //--------------------------------
  
  
  //uint32_t ep=NVIC_EncodePriority (2, 3, 3);
  //NVIC_SetPriority(SysTick_IRQn, ep);
  
  //uint32_t gr=NVIC_GetPriorityGrouping();
  
  //���������� ���������� �������� ������ �������
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;//R shift
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)R_OFFSET_TIM_IT_PRIO;// priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)R_OFFSET_TIM_IT_PRIO_SUB;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /*NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;//G shift
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)G_OFFSET_TIM_IT_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)G_OFFSET_TIM_IT_PRIO_SUB;
  NVIC_Init(&NVIC_InitStructure);*/
  
 /* NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;//B shift
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)B_OFFSET_TIM_IT_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)B_OFFSET_TIM_IT_PRIO_SUB;
  NVIC_Init(&NVIC_InitStructure);*/
  
  /*
  //������������ 168 MHz
  Timer_Config(2,RCC_APB2Periph_TIM8,
               TIM8,
               3,42000,
               TIM_CounterMode_Down,
               TIM8_UP_TIM13_IRQn,
               false);//��������� ��������� ������� ������������
  
  //��������� ���������� ��������� �������
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;//TIM6_DAC_IRQn;//PLAY TIMER 84MHz
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)PLAY_TIMER_IT_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)PLAY_TIMER_IT_PRIO_SUB;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);
  TIM8->CR1 &= (uint16_t)~TIM_CR1_ARPE;// Reset the ARR Preload Bit
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn); 
  TIM_Cmd(TIM8,ENABLE);//������ ���������,��������� � �����������
  return;*/
  
  //��������� ���������� ������������� �������
  /*NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)INTERVAL_TIMER_IT_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)INTERVAL_TIMER_IT_PRIO_SUB;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), PLAY_TIMER_IT_PRIO,PLAY_TIMER_IT_PRIO_SUB));
  
  
  SysTick->LOAD  = ((uint32_t)168000 & SysTick_LOAD_RELOAD_Msk) - 1;  // set reload register
  SysTick->VAL   = 1680000;//(uint32_t)(StartingIntervalPeriod); // Load the SysTick Counter Value 
  SysTick->CTRL  =  SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;               // Enable SysTick IRQ and SysTick Timer 
}
//----------------------------------------------------------------------------//
static uint32_t SampleIntervalPeriod=16800;//10p/Ms
//�������� ������ ������������,1 ��
void TIM8_UP_TIM13_IRQHandler()//TIM6_DAC_IRQHandler()
{
  GREEN_LED_ON();
}  
//----------------------------------------------------------------------------//  
void set_setup_center(uint8_t r,uint8_t g,uint8_t b){
  _setDACs(2047,2047);
  TIM1->CCR2=G_PWM[G=g];
  TIM1->CCR1=R_PWM[R=r];
  TIM1->CCR3=B_PWM[B=b];
}
//******************************************************************************************************// 
//******************************************************************************************************//  
//������������ ��������������� ������ ������������
//******************************************************************************************************// 
//******************************************************************************************************// 
void SysTick_Handler(void)
{
  //SysTick->CTRL  = (uint32_t)0;//������ ������ �������� �������(����)
  ICP* p=0;
  //static int cnt=0;
    
  int f=*(&freesize);
  
  if(ICmode==IM_WLAN){
     
    if(stream_mode&2){
      if(f>=SPI_DMA_BLOCKx2){//STREAMBUF_SIZE/*-SPI_DMA_BLOCK_SIZE/2*/){//��������+�����
        wrPtr+=SPI_DMA_BLOCK_SIZE;if(wrPtr>=StreamOverbuf)wrPtr=StreamBuf;
        f-=SPI_DMA_BLOCK_SIZE;*(&freesize)=f;
        stream_mode=1;
        DMA2_Stream2->M0AR = (uint32_t)wrPtr;
        DMA2_Stream2->NDTR = (uint32_t)SPI_DMA_BLOCK_SIZE;
        DMA2_Stream2->CR |= (uint32_t)DMA_SxCR_EN; //RX enable
        SPI1_HAND_HIGH;
      }
    }
    else if(!stream_mode){// && PlayFlag){
      PlayFlag=0;LScanStatus=STATUS_STOP;wrPtr=rdPtr=StreamBuf;
    }
    //if(f>=SPI_DMA_BLOCKx2){
    // SPI1_HAND_LOW;//too big delay
    //}
    
  }//------------------------------------------------------------------------
  else { //SD
    //1-bufer ready to fill (it->process)
    //2-block copyed to stream(process->it)
    if(f>=SPI_DMA_BLOCK_SIZE){
      if(rdy_ilda&2){
        wrPtr+=SPI_DMA_BLOCK_SIZE;if(wrPtr>=StreamOverbuf)wrPtr=StreamBuf;
        f-=SPI_DMA_BLOCK_SIZE;*(&freesize)=f;
        rdy_ilda&=0;
        PlayFlag=1;LScanStatus=STATUS_PLAY;
      }
      
      if(f>=SPI_DMA_BLOCK_SIZE && !rdy_ilda){
        rdy_ilda=1;
      }
    }

  }
  //===========================================================
  
  if(!PlayFlag){
    ORANGE_LED_ON();
    return;
  }
  //---------------------
    
  p=(ICP*)rdPtr;
  
  rdPtr+=sizeof(ICP);if(rdPtr>=StreamOverbuf)rdPtr=StreamBuf;
  
  #if (1)
    f+=8;*(&freesize)=f;
  
  #else
    cnt_icp+=sizeof(ICP);
    if(cnt_icp>=SPI_DMA_BLOCK_SIZE){
      cnt_icp=0;
      //freesize+=SPI_DMA_BLOCK_SIZE;
      f+=SPI_DMA_BLOCK_SIZE;*(&freesize)=f;
    }
  #endif
  
  
  
  if(rdPtr==wrPtr && f==STREAMBUF_SIZE){
    PlayFlag=0;LScanStatus=STATUS_STOP;
  }

  /*
  cnt_icp+=sizeof(ICP);
  if(cnt_icp>=SPI_DMA_BLOCK_SIZE){
    cnt_icp=0;
    f+=SPI_DMA_BLOCK_SIZE;
    *(&freesize)=f;
  }*/
  
  
  //������������ ������(��������,���� ���������� ��� ��������� ������������)
  //if(discr_MS!=62){HardFault_Handler();}//DBG
  if(!p->discrMs || p->cmd)
  {
    //ORANGE_LED_ON();//��������� ������
    RED_LED_ON();
    return;//goto load_sys_interval;
  }

  //------------------
  X=(((uint16_t)p->hxhy&0x0f)<<8)|(uint16_t)(p->lx);
  Y=(((uint16_t)p->hxhy&0xf0)<<4)|(uint16_t)(p->ly);
  _setDACs(X,Y);//��������� ���������
  //------------------
  TIM1->CCR1=R_PWM[R=p->r];TIM1->CCR2=G_PWM[G=p->g];TIM1->CCR3=B_PWM[B=p->b];
  GREEN_LED_ON();
  //================================
//load_new_sys_interval:
  #ifdef USE_SPD_CORRECTOR
   if(f>STREAMBUF_SIZE_DIV2){//STREAMBUF_SIZE_3_4){
     if(sampleMult<120)sampleMult++;  //SLOWLY
   }
   else {
     //if(ICmode==IM_WLAN){
     //  if(sampleMult>80 && f<=STREAMBUF_SIZE-SPI_DMA_BLOCK_SIZE)sampleMult--;//FASTER realtime stream
     //}
     //else{
       if(sampleMult>100 && f<=STREAMBUF_SIZE_DIV2)sampleMult--;//FASTER not more than 100 !!!
     //}
   }
    
   uint32_t nSampleIntervalPeriod=1680*sampleMult/p->discrMs; 
  #else
   uint32_t nSampleIntervalPeriod=168000/p->discrMs; 
  #endif
  
  if(nSampleIntervalPeriod==SampleIntervalPeriod)return;
  SampleIntervalPeriod=nSampleIntervalPeriod;
//load_sys_interval:
  SysTick->LOAD  = ((SampleIntervalPeriod-1) & SysTick_LOAD_RELOAD_Msk);// - 1;  // set reload register
  SysTick->VAL   = SampleIntervalPeriod;                                // Load the SysTick Counter Value 
  SysTick->CTRL  = /*SysTick_CTRL_CLKSOURCE_Msk|*/SysTick_CLKSource_HCLK|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;// Enable SysTick IRQ and SysTick Timer 
 
  
}
//----------------------------------------------------------------------------//
/*
Only the DMA2 controller is able to perform memory-to-memory transfers.
������ ���������/��������� ��������� �� ������� 4
�������� DMAy Streamx's pending flags.
param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
to 7 to select the DMA Stream.
_D -���������� _S-�������� _N -������ ��� �����������
*/
void DMA2_memcpy(DMA_Stream_TypeDef* DMAy_Streamx,void * _D, const void * _S, size_t _N)
{
  //�������� DMA
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  uint8_t nev=_N&1;//�����  
  DMA2_Stream1->CR = 0;
  while(DMA2_Stream1->CR & DMA_SxCR_EN);
    
  DMA2_Stream1->PAR =   (uint32_t)_S ; 
  DMA2_Stream1->M0AR =  (uint32_t)_D ; 
  DMA2_Stream1->NDTR =  (uint32_t)nev?_N:(_N/4); 
  DMA2_Stream1->FCR = DMA_SxFCR_DMDIS;//FIFO disable
  DMA2_Stream1->CR = //DMA_SxCR_EN|DMA_SxCR_DIR_1|DMA_SxCR_PL_1|DMA_SxCR_PINC|DMA_SxCR_MINC|
   //(nev?0:DMA_SxCR_PSIZE_1|DMA_SxCR_MSIZE_1|DMA_SxCR_PBURST_0);
     DMA_SxCR_EN|DMA_SxCR_DIR_1|DMA_SxCR_PL|DMA_SxCR_PINC|DMA_SxCR_MINC;
  //� ������ ������ ��������� �� 4
}   
//----------------------------------------------------------------------------//
//������������� ���� � ������ ����� ����������
FRESULT Sound_notification(float volume,TCHAR* p_file_name)
{
  if(TMMask & TM_NOVOICE)return FR_OK;
  if(volume)SoundGain=volume;
  return PlayMP3File(p_file_name);
}
//----------------------------------------------------------------------------//
void uDelay (uint32_t usec)
{
  uint32_t count = 0;
  const uint32_t utime = (120 * usec / 7);
  do
  {
    if ( ++count > utime )
    {
      return ;
    }
  }
  while (1);
}


void mDelay (uint32_t msec)
{
  uDelay(msec * 1000);   
}
/******************* (C) COPYRIGHT 2016 dem1305 *****END OF FILE****/
