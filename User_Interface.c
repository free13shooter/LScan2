/**
  ******************************************************************************
  * @file    User_Button.c 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    26-Yul-2017
  * @brief   user buttons processing
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "LScan.h"
//------------------------------------------------------------------------------
//USER FRONTAL PANEL BUTTONS: PA8, PD9-PD15 
//отсчеты по кнопкам и линии детектирования MMC
uint8_t button[9]={0,0,0,0,0,0,0,0,0};//8 buttons+MMC detect
uint8_t buttonRep[8]={0,0,0,0,0,0,0,0};
static uint8_t button_TCK[9]={0,0,0,0,0,0,0,0,0};

extern bool SDCardMounted;
//------------------------------------------------------------------------------	
//---------------настройка пинов кнопок управления-------------------------
void Buttons_config()
{
  /*настройка порта
    настройка portsource
    настройка EXTI
    настройка IT*/
  //memset(button,0,9);memset(button_TCK,0,9);
  //PA8 , PD9-PD15
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //режим-альтернативная функция
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP ;//подтяжка вверх
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_25MHz;//тактирование пинов
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|
    GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  //Sleep(500);
  /*
  //SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource9);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource12);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13);
  //SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource14);
  //SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource15);
  
  EXTI_ClearITPendingBit(EXTI_Line15|EXTI_Line14|EXTI_Line13|EXTI_Line12||EXTI_Line11|EXTI_Line10|EXTI_Line9|EXTI_Line8);
  
  EXTI_InitTypeDef exti;
  EXTI_StructInit(&exti);
  exti.EXTI_Line =EXTI_Line8|EXTI_Line9|EXTI_Line10|EXTI_Line11|EXTI_Line12|EXTI_Line15;//|EXTI_Line13| EXTI_Line14;
  exti.EXTI_Mode = EXTI_Mode_Interrupt;    
  // срабатываем в зависимости от аргумента задачи (фронт,спад)
  exti.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling; 
  exti.EXTI_LineCmd = ENABLE;    // вкл
  EXTI_Init(&exti);
  //
    
  NVIC_DisableIRQ(EXTI9_5_IRQn); NVIC_DisableIRQ(EXTI15_10_IRQn); 
  
  NVIC_InitTypeDef NVIC_InitStructure;
  //приоритет прерываний
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)BUTTONS_EXTI_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)BUTTONS_EXTI_PRIO_SUB;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_EnableIRQ(EXTI15_10_IRQn);//EXTI15_10_IRQn=SD_DETECT_EXTI_IRQChannel; 
  NVIC_EnableIRQ(EXTI9_5_IRQn);*/
  
  //SystemTimer* BUT_timer=SetPeriodicSystemTimer((uint16_t)BUTTON_REP_MS,&ButtonsScanFunc);
  
  HANDLE BUT_thread=CreateThread(&ButtonsScanFunc,0,TM_PRIV|TM_PSP,0);
}
//------------------------------------------------------------------------------
//проверка кнопки.Если установлен флаг,вернет true
bool CheckButtonLine(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin,uint8_t butIndex)
{
  bool rs=false;
  
  //PA8 PD9 PD10 PD11 PD12 PD13 PD14 PD15 + PA15(MMC)
  
  if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin)==RESET)
  {
    if(button_TCK[butIndex]==0 && (butIndex!=8 || (butIndex==8 && !SDCardMounted)) )
    {
      Beep(1000+butIndex*500,50);
      
      if(button[butIndex]<BUTTONS_REP)
      {
        if(butIndex==8)
        {
          if(button[butIndex]==0)button[butIndex]=1;
        }
        else 
        {
          button[butIndex]++;
        }
      }
      
      button_TCK[butIndex]=button[butIndex]==1?BUTTONS_TCK:BUTTONS_TCK/2;
    }
    
  }
    
  rs= true;
  
  return rs;
}
/*
void EXTI9_5_IRQHandler()//buttons 0,1
{
  //but0=PA8
  CheckButtonEXTILine(EXTI_Line8,GPIOA, GPIO_Pin_8,0);
   
  //but1=PD9
  CheckButtonEXTILine(EXTI_Line9,GPIOD, GPIO_Pin_9,1);
}

void EXTI15_10_IRQHandler()
{
  ITStatus sb0=EXTI_GetITStatus(EXTI_Line8);
  ITStatus sb1=EXTI_GetITStatus(EXTI_Line9);
  ITStatus sb2=EXTI_GetITStatus(EXTI_Line10);
  ITStatus sb3=EXTI_GetITStatus(EXTI_Line11);
  ITStatus sb4=EXTI_GetITStatus(EXTI_Line12);
  ITStatus sb5=EXTI_GetITStatus(EXTI_Line13);
  ITStatus sb6=EXTI_GetITStatus(EXTI_Line14);
  ITStatus sb7=EXTI_GetITStatus(EXTI_Line15);
  //but2=PD10
  CheckButtonEXTILine(EXTI_Line10,GPIOD, GPIO_Pin_10,2);
  //but3=PD11
  CheckButtonEXTILine(EXTI_Line11,GPIOD, GPIO_Pin_11,3);
  //but4=PD12
  CheckButtonEXTILine(EXTI_Line12,GPIOD, GPIO_Pin_12,4);
  //but5=PD13
  CheckButtonEXTILine(EXTI_Line13,GPIOD, GPIO_Pin_13,5);
  //but6=PD14
  CheckButtonEXTILine(EXTI_Line14,GPIOD, GPIO_Pin_14,6);
  //but7=PD15
  if(CheckButtonEXTILine(EXTI_Line15,GPIOD, GPIO_Pin_15,7))
  {
    //detectMMC=PA15
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)==RESET)
    {
      if(button_TCK[8]==0)button[8]=1;
      button_TCK[8]=BUTTONS_TCK;
    }
        
    button_TCK[8]=BUTTONS_TCK;
  }
}*/
//------------------------------------------------------------------------------	
int ButtonsScanFunc(void* pargs)
{
  uint32_t pin;
  
loop_scan_buttons:
  //---------------- 
  pin=GPIO_Pin_8;
  for(int i=0;i<8;pin*=2,i++)
  {
    if(button_TCK[i]>0)button_TCK[i]--;
    
    CheckButtonLine(pin==GPIO_Pin_8?GPIOA:GPIOD,pin,i);
  }
  
  
  if(button_TCK[8]>0)button_TCK[8]--;
  CheckButtonLine(GPIOA,GPIO_Pin_15,8);
    
  if((button[8]==1 && !SDCardMounted)||(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)==SET && SDCardMounted) )
  {
    button[8]=0;
    //SD_detect_processing();
  }
  
  /*
  bool sta0=  CheckButtonLine(GPIOA,GPIO_Pin_8,0);
  bool sta1=  CheckButtonLine(GPIOD,GPIO_Pin_9,1);
  bool sta2=  CheckButtonLine(GPIOD,GPIO_Pin_10,2);
  bool sta3=  CheckButtonLine(GPIOD,GPIO_Pin_11,3);
  bool sta4=  CheckButtonLine(GPIOD,GPIO_Pin_12,4);
  bool sta5=  CheckButtonLine(GPIOD,GPIO_Pin_13,5);
  bool sta6=  CheckButtonLine(GPIOD,GPIO_Pin_14,6);
  bool sta7=  CheckButtonLine(GPIOD,GPIO_Pin_15,7);
  bool staDet=CheckButtonLine(GPIOA,GPIO_Pin_15,8);
  
  if(sta0||sta1||sta2||sta3||sta4||sta5||sta6||sta7||staDet)
  {
    int n=0;
    int b=n;
  }
  
  for(uint16_t i=0;i<9;i++)
  {
    if(button_TCK[i]>0)button_TCK[i]--;
  }
  */
  
  Sleep(BUTTON_REP_MS);
  
  goto loop_scan_buttons;
}

//------------------------------------------------------------------------------
bool isButtonPressed(uint8_t butIndex)
{ 
   
  if(button[butIndex]){button[butIndex]=0;return true;}
 
  
  return false;
}
//------------------------------------------------------------------------------
bool isButtonPressedLong(uint8_t butIndex)
{ 
  if(button[butIndex] == BUTTONS_REP )//удерживалась
  {
    button[butIndex]=0;return true;
  }  
  
  return false;
}
//------------------------------------------------------------------------------
bool isButtonPressedShort(uint8_t butIndex)
{ 
  if(button[butIndex] && button[butIndex]<BUTTONS_REP )//не удерживалась
  {
    button[butIndex]=0;return true;
  }  
  
  return false;
}
//------------------------------------------------------------------------------
bool butEventClick(uint8_t butIndex) //не обнулять
{ 
  if(button[butIndex]){return true;}
  return false;
}
//------------------------------------------------------------------------------
bool butEventPress(uint8_t butIndex)//не обнулять
{ 
  bool rs=false;
  
  uint16_t pin=GPIO_Pin_8;
  if(butIndex)pin=pin<<(butIndex);

  uint8_t sta=GPIO_ReadInputDataBit((butIndex==0?GPIOA:GPIOD), pin);
  
  if(sta==SET && button[butIndex])
  {
    return true;
  }
    
  return rs;
}
//------------------------------------------------------------------------------
