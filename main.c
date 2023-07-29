/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sdcard.h"
#include "core.h"
#include "core_cmFunc.h" //asm funcs
//------------------------------------------------------------------------------


//#pragma data_alignment = 4   
//__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


uint16_t PrescalerValue = 0;

__IO uint8_t UserButtonPressed = 0;

extern uint16_t testDelay;
extern uint16_t testStep;

//extern volatile uint8_t PlayFlag;
//extern uint32_t FullSysReset;
//extern uint32_t USB_Disconnect;

extern FATFS FileSysObj;

//------------------------------------------------------------------------------
extern uint8_t R;extern uint8_t G;extern uint8_t B;	  //целевые цвета 
extern uint16_t SampleDiscrInMs;//текущая дискретизация для 1 мс-семпла

extern FATFS* FS_SDIO_disk;
extern FATFS* FS_SPI_disk;

extern bool SDCardMounted;

extern bool repeatMode;

extern float SoundGain;

extern IMODE ICmode;//тип соединения
//------------------extern STask* U2DMA_task;//системная задача наблюдения за DMA UART2

extern SERVO_PAR wndServo;//серво фронтальной панели,окно 
extern bool Power;//состояние питания
extern uint16_t WndGrad;//0=окно открыто 180=закрыто
unsigned int* ptTask_CurrentTask;
extern uint32_t errors;
//--------------------------MAIN----------------------------------------------//
//наблюдение за состоянием

extern SystemTimer* Power_timer;
extern SystemTimer* U2DMA_timer;
HANDLE PlayerThread=NULL;
extern volatile bool Exit_Proccess_Player;
extern volatile HTHREAD pctcon;
bool OnSDMounted=false;//событие
extern unsigned char ServoOffPwrCycles;
Event* pEv;
extern volatile TYPEMODEMASK TMMask;    //маска настроек режимов воспроизведения файлов,LScan.c

#define _voiceenable (!(TMMask & TM_NOVOICE))
//----------------------------------------------------------------------------//
int ButtonThreadFunc(void* ctx)
{
  //isButtonPressed(but_power)||(OnSDMounted && !Power);
  //FRESULT ffr;//=f_chdrive (_T("0:"));  
  //Wnd_OPEN();playILDAFile("eff/eff001.ild");
//blop_tst:
  //char s[]="madonna-jump-radio-edit.mp3";
  //ffr=PlayMP3File("0://madonna.mp3");
  //ffr=PlayMP3File("0://madonna-jump-radio-edit.mp3");
  //FIL* pf=mmmalloc(sizeof(FIL));
  //ffr=f_open(pf,"0://madonna-jump-radio-edit.mp3",FA_READ);
  //FIL* fn=mmmalloc(sizeof(FIL));  /* File object */
  //ZeroMemory(fn,sizeof(FIL));//по-любому теряем файл,можно не закрывать
  //ffr=f_open(&fn,"madonna.mp3",FA_READ);

  //LED_PanicLoop(2);
  //Sleep(1000);
  //LED_PanicLoop(4);
  //SetEvent((HANDLE)pEv);
  //goto blop_tst;
  //PlayerThread=0;
  //EnterCriticalSection;
  
  /*
  
  uint32_t i=0;
  char* pbuf=rxbuf;memset(rxbuf,0,64);
  BitAction st0;
  GPIOB->BSRRL = GPIO_Pin_8;//handshake LOW
  bool sw=false;
    
  while(1){
   
   SPI1->CR1 &= SPI_NSSInternalSoft_Reset;
   
      
   do{
     SPI1->CR1 &= SPI_NSSInternalSoft_Reset;
     
     while(GPIOA->IDR & GPIO_Pin_15);//wait CS
     GPIOB->BSRRH = GPIO_Pin_8;//handshake HIGH;
     
     uint32_t is=0;
     
     while((!(SPI1->SR & SPI_SR_RXNE))){
       if((GPIOA->IDR & GPIO_Pin_15) || ++is>168000){
         if(is>160000)memset(rxbuf,0,64);
         goto brk_rd;
       }
       GPIOB->BSRRH = GPIO_Pin_8;//handshake HIGH;
     }
          
           rxbuf[i++]=(char)SPI1->DR;//SPI_I2S_ReceiveData(SPI3); // read recieived
     if(i>=64){
       i=0;//i>>=7;//i&=~i;
     }
  
        
   }while(!(GPIOA->IDR & GPIO_Pin_15));//GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)==RESET);
      
  brk_rd:
   SPI1->CR1 |= SPI_NSSInternalSoft_Set;
   i=0;
   
   GPIOB->BSRRL = GPIO_Pin_8;//handshake LOW
  }
*/
  
  
  return 1;
}

//____________________________________________________________________________________________________________//

int main(void)
{
  FRESULT fr=FR_OK;
  LScan_Init();
  Power_OFF();
  //Wnd_CLOSE();
  
#ifdef DISCOVERY_BOARD
  
  uint8_t rr=0;uint8_t gg=10;uint8_t bb=50;
  
  while(1){
    Sleep(200);
    set_setup_center(rr+=5,gg+=15,bb+=20);
    if(bb>=150)bb=0;if(gg>=50)gg=0;if(rr>=200)rr=0;
    if(errors) {
      //Sleep(1000);
      if(errors==ERR_DATA_ZERO)errors=ERR_DATA_ZERO_PROCESSED;
    }
    GREEN_LED_OFF();BLUE_LED_OFF();RED_LED_OFF();ORANGE_LED_OFF(); 
  }
#endif 
  
  //PlayerThread=CreateThread((TFUNC*)&ButtonThreadFunc,0,0,0);//TM_PRIV|TM_PSP,0);

  //Power_ON();
  //Wnd_OPEN();

  FS_SDIO_disk=mmmalloc(sizeof(FATFS));
  FS_SPI_disk=mmmalloc(sizeof(FATFS));
  ZeroMemory(FS_SDIO_disk,sizeof(FATFS));
  ZeroMemory(FS_SPI_disk,sizeof(FATFS));
    
  FRESULT rspi0=f_mount(FS_SPI_disk,_T("0:/"),1);
  FRESULT rspi1=f_mount(FS_SDIO_disk,_T("1:/"),1);
  SDCardMounted=false;

  SetSoundGain(0.2f);
  //Power_ON();
  
  //fr=Sound_notification(1,"0://madonna.mp3");
  //fr=Sound_notification(1,"0://SOUNDS/ot_vinta.mp3");
  //while(1);
  //-------------------------
  //CreateTask((TFUNC*)&tst_func,0);
  //PlayerThread=CreateThread((TFUNC*)&tst_func,0,0,0);//TM_PRIV|TM_PSP,0);
/*
lp_1:  
  if(!PlayerThread)PlayerThread=CreateThread((TFUNC*)&tst_func,0,0,0);//TM_PRIV|TM_PSP,0);
  IS_RES  PlayState=playILDAFile("0://eff/eff001.ild");
  //SetSoundGain(0.9f);
  //Proccess_Player();
  */
  //Power_ON();
  //fr=PlayMP3File("0://madonna-jump-radio-edit.mp3");
  //fr=PlayMP3File("0://SOUNDS/sd_detected.mp3");
  //goto lp_1;
  
  //while (1);
  //======стартовая инфа========
  //Wnd_OPEN();Sleep(500);
  //for(int no=0;no<10;no++){RECT rloc={-4000,0,200,200};OutText(&rloc,"LScan2 intercon WLAN: ...",0xFFFF00);}
  //for(int no=0;no<10;no++){RECT rloc={-4000,0,200,200};OutText(&rloc,"Password: inferno1 ...",0x00FF00);}
  //for(int no=0;no<30;no++){RECT rloc={-4000,0,150,150};OutText(&rloc,"connect: 192.168.13.2:2264,",0x00FFFF);}
  //for(int no=0;no<30;no++){RECT rloc={-4000,0,150,150};OutText(&rloc,"Web connect: 192.168.13.2:2265",0xFFFFFF);}
    
  Sleep(1000);
  
  //============================
  //---------------------------main loop---------------------------------------//
  Atom32Set((unsigned long*)&PlayerThread,0);
  unsigned long pt=0;
  //OnSDMounted=
  //SDCardMounted=0;
  
  //CreateThread((TFUNC*)&ButtonThreadFunc,0,0,0);
  
  bool starting=true;
    
  while(1)
  {
    ADC_check_complete();
    
    if(ICmode==IM_NO)ICmode=IM_WLAN;
    
    bool OnSDMounted=SD_detect_processing();
    
    if(isButtonPressed(but_operate))
    {
      pt=TMMask ^ (TYPEMODEMASK)TM_FILE_RANDOM ;// инвертируем TM_FILE_RANDOM бит
      if(pt & TM_FILE_RANDOM)Sound_notification(1,"0://SOUNDS/random_file.mp3");else Sound_notification(1,"0://SOUNDS/all_files.mp3");
      Atom16Set((unsigned short*) &TMMask,pt);
    }
    else if(isButtonPressed(but_rec))
    {
      pt=TMMask ^ (TYPEMODEMASK)TM_NOVOICE ;// инвертируем TM_NOVOICE бит
      if(pt & TM_NOVOICE)fr=PlayMP3File("0://SOUNDS/modes_voice_off.mp3");else fr=PlayMP3File("0://SOUNDS/modes_voice_on.mp3");
      Atom16Set((unsigned short*) &TMMask,pt);
    }
    else if(isButtonPressed(but_eject))//оптимизация
    {
      pt=TMMask ^ (TYPEMODEMASK)TM_OPT ;// инвертируем TM_OPT бит - изменить озвучку
      if(pt & TM_OPT){
        if(bRnd)Sound_notification(1,"0://SOUNDS/otryv.mp3");
        else if(bRnd)Sound_notification(1,"0://SOUNDS/ot_vinta.mp3");
        else Sound_notification(1,"0://SOUNDS/hello_people.mp3");
      }
      else {
        if(bRnd)Sound_notification(1,"0://SOUNDS/beregite_glaza.mp3");
        else if(bRnd)Sound_notification(1,"0://SOUNDS/high_power.mp3");
        else Sound_notification(1,"0://SOUNDS/koroche.mp3");
      }
      Atom16Set((unsigned short*) &TMMask,pt);
    }
    else if(SDCardMounted && !OnSDMounted && LScanStatus==STATUS_STOP  && isButtonPressed(but_play))
    {
      Power_ON();
      ICmode=IM_SD;
      if(Power_timer)DeleteSystemTimer(&Power_timer);
      fr=f_mount(FS_SDIO_disk,_T("1:/"),1);
      if(fr!=FR_OK)
      {
        f_mount(NULL,_T("1:/"),0);//unmount
        Sound_notification(1,"0://SOUNDS/error.mp3");
        Sound_notification(1,"0://SOUNDS/ne_poluchaetca_init.mp3");
        Sound_notification(1,"0://SOUNDS/proverte_kartu_pamyati.mp3");
        SDCardMounted=false;
        ICmode=IM_WLAN;
      }
      if(!PlayerThread){
        pt=Atom32Set((unsigned long*)&PlayerThread,(unsigned long)CreateThread((TFUNC*)&Proccess_Player,0,0,0));  //self-terminated
        if(!pt)while(1);
      }
    }
    
    
      
    if(isButtonPressed(but_power)||(OnSDMounted && !Power))
    {
            
      if(Power)
      {
        if(Power_timer)DeleteSystemTimer(&Power_timer);
        pt=Atom32Get((unsigned long*)&PlayerThread);
        if(pt!=0)
        {
          Exit_Proccess_Player=true;
          do{
          pt=Atom32Get((unsigned long*)&PlayerThread);
          Sleep(100);
          }
          while (pt);
        }
        
        LScanStatus=STATUS_STOP;
        SetRGB(0,0,0);
        Sound_notification(1,"0://SOUNDS/power_off.mp3");
        Sleep(200);
        Power_off_timer_func();
        Atom8Set(&LScanStatus,STATUS_STOP);
        ICmode=IM_NO;
        f_mount(NULL,_T("1:/"),0);//unmount
      }
      else //POWER ON
      {
        Power_ON();
        ICmode=IM_WLAN;
        
        if(_voiceenable)
        {
          if(starting)
          {
            starting=false;
            
            //if(bRnd)Sound_notification(1,"0://SOUNDS/hello_people.mp3");
            //if(bRnd)Sound_notification(1,"0://SOUNDS/otryv.mp3");
            //if(bRnd)Sound_notification(1,"0://SOUNDS/ot_vinta.mp3");
            
            //if(bRnd)
            Sound_notification(1,"0://SOUNDS/danger_laser.mp3");
            //else if(bRnd){Sound_notification(1,"0://SOUNDS/beregite_glaza.mp3");
            Sound_notification(1,"0://SOUNDS/high_power.mp3");
            //    else if(bRnd)Sound_notification(1,"0://SOUNDS/koroche.mp3");
          }
          else Sound_notification(1,"0://SOUNDS/power_on.mp3");
          
          if(SDCardMounted)fr=Sound_notification(1,"0://SOUNDS/sd_detected.mp3");
        }
      }
    }
    else
    {
      unsigned char sc= Atom8Get(&ServoOffPwrCycles);
      if(sc)
      {
        Sleep(100);
        sc--;
        Atom8Set(&ServoOffPwrCycles,sc);
        if(sc==0)//откл.управление серво
        {
          ServoPinConfig(false);
        }
      }
    }
    
    //if(ICmode==IM_WLAN && U2DMA_timer==NULL)U2DMA_timer=SetPeriodicSystemTimer((uint16_t)1000,&U2timerProc);

    if(SDCardMounted && OnSDMounted)
    {
      pt=Atom32Get((unsigned long*)&PlayerThread);
      if( pt==NULL && Power)
      {
        Sleep(250);//антидребезг
        
        fr=f_mount(FS_SDIO_disk,_T("1:/"),1);
        if(fr!=FR_OK)
        {
          f_mount(NULL,_T("1:/"),0);//unmount
          Sound_notification(1,"0://SOUNDS/ne_poluchaetca_init.mp3");
          Sound_notification(1,"0://SOUNDS/proverte_kartu_pamyati.mp3");
          SDCardMounted=false;
          ICmode=IM_WLAN;
        }
        else 
        {
          Exit_Proccess_Player=false;
          ICmode=IM_SD;
          if(Power_timer)DeleteSystemTimer(&Power_timer);
          Sound_notification(1,"0://SOUNDS/mSD.mp3");
          pt=Atom32Set((unsigned long*)&PlayerThread,(unsigned long)CreateThread((TFUNC*)&Proccess_Player,0,0,0));  //self-terminated
          if(!pt)while(1);
        }
      }
    }
    //else {
    //  ICmode=IM_WLAN;
    //}
    //--------------------------------------
    
      
    switch(Atom8Get(&LScanStatus))
    {
    case STATUS_PLAY:
    case STATUS_PAUSE:
      if(Power_timer)DeleteSystemTimer(&Power_timer);
      if(!Power)Power_ON();
      if(WndGrad!=OPEN_ANGLE)Wnd_OPEN();
      break;
    
    case STATUS_STOP:
      //RED_LED_ON();//индикация отсутствия данных
      SetRGB(0,0,0);
    default://break;
      if(!Power_timer)Power_timer= SetOnePulseSystemTimer(10000,&Power_off_timer_func);
    
      Sleep(100);
    }
    //----------ПРОВЕРКА ИЗМЕНЕНИЯ СОСТОЯНИЯ ФЛЕШ--------------------------
    //uint8_t mm3[8]={mm2[0],mm2[1],mm2[2],mm2[3],mm2[4],mm2[5],mm2[6],mm2[7]};
    //uint8_t* pmm3=mm3;
    /*
    
    if(FSMounted!=SDCardMounted)
    {
      Delay((uint32_t)80);//антидребезг
      
      if(SDCardMounted)
      {
        //----------
        int attempts=1; //попыток монтирования карты
        while(SDCardMounted&&(!FSMounted&&attempts))
        {
          --attempts;
          FRESULT rs=f_mount(&FileSysObj,_T("1:/"),0);
          Delay((uint32_t)80);//антидребезг
          if(rs==FR_OK)//смонтировано
          {
            FSMounted=true;
            //ModeUSB=false;-только при запуске файла!!!
            uint16_t mel[6]={2200,90,0,90,2200,90};memcpy(mel_of_flash_detect,mel,12);
            BeepMelody(mel_of_flash_detect,3);
            //смонтировано,перейти к поиску и проигрыванию первого найденного файла ILDA
            //-----------------
            if(ICmode!=IM_SD)
            {
              if(ICmode==IM_USB)Disconnect_FS_USB_Device(&USB_OTG_dev);
              ICmode=IM_SD;
            }
            OpenSecWindow(75);
            int i=500000;
            while(--i){Hardware_Monitoring(false);}//пауза
            //Init_Player();
            for(int no=0;no<20;no++){RECT rloc={-1000,0,200,200};OutText(&rloc,"Flash mode -->",0x00FFFF);}
            //-----------------
          }
          else
          {
            if(FSMounted){f_mount(NULL,_T("1:/"),0);}//unmount
            FSMounted=false;
            //DeInit_Player();
          }
        }//while
    
        if(!FSMounted)//не удалось
        {
          if(FSMounted){f_mount(NULL,_T("1:/"),0);}//unmount
          FSMounted=SDCardMounted=false;
          //DeInit_Player();
          uint16_t mel[6]={2000,100,1800,100,1600,100};memcpy(mel_of_flash_detect,mel,12);
          BeepMelody(mel_of_flash_detect,3);
        }
        
      }//if(SDCardMounted)
      else//card removed from slot
      {
       uint16_t mel[6]={1200,100,0,100,1200,100};memcpy(mel_of_flash_detect,mel,12);
       if(FSMounted){f_mount(NULL,_T("1:/"),0);}//unmount
       SDIO_ClockCmd(DISABLE);
       FSMounted=false;
       BeepMelody(mel_of_flash_detect,3);
       //DeInit_Player();
       if(ICmode==IM_SD)
       {
         Soft_Connect_USB_Device(&USB_OTG_dev);
         ICmode=IM_USB;
       }
      }
      
    }
    //__________________________________________________________________________//
    if( FSMounted && LScanStatus == STATUS_STOP )
    {
      if(repeatMode==true || (butstate & BUT_MSK)!=0 )//запуск любой кнопкой 
      {
        if(repeatMode==false)
        {
          Beep(2200,300);
          Delay(PRE_KEY_DELAY);
          UINT cntrep=2500;
          while((butstate & BUT_MSK)==((uint8_t)BUT_UP|BUT_DN) && SD_Detect())//пока не отжаты
          {
            Hardware_Monitoring(true);Delay(5000);
            if(cntrep>0)//режим повтора при длинном нажатии
            {
              cntrep--;
              if(cntrep==0){ Beep(2000,500);repeatMode=((bool)(!repeatMode));Delay(80);}
            }
          }
        } //if(repeatMode==false)
        //PLAY +++++++++++++++
        if(Power_timer)DeleteTimer(&Power_timer);
        if(!Power)Power_ON();
        if(WndGrad>0)Wnd_OPEN();
        
        IS_RES rs=Proccess_Player();//начать заново
        //check error
        if(rs&IS_ERR==IS_ERR)//ошибка или файлов для проигрывания не найдено.Вернуть режим USB
        {
          FSMounted=SDCardMounted=false;
          SetRGB(0,0,0);
          //DeInit_Player();
          if(ICmode==IM_SD)
          {
            Soft_Connect_USB_Device(&USB_OTG_dev);
            ICmode=IM_USB;
          }
        }
      }
    }*/
    switch(errors){
    case ERR_DATA_ZERO:
      errors=ERR_DATA_ZERO_PROCESSED;
      Sound_notification(1,"0://SOUNDS/ERR_DATA_ZERO.mp3");
      //SetOnePulseSystemTimer(6000,&NVIC_SystemReset);
      break;
    case ERR_FREE_NEGATIVE:
      Sound_notification(1,"0://SOUNDS/ERR_FREE_NEGATIVE.mp3");
      SetOnePulseSystemTimer(6000,&NVIC_SystemReset);
      break;
    }
    //__________________________________________________________________________//
    //if(UserButtonPressed){UserButtonPressed=0;UsrButPressed();}//debug
    
    if(FullSysReset) //soft reset MCU
    {
      LSCAN2_LEDOn(LEDWHITE);//WHITE ,DEBUG
      //while(1);          //DEBUG
      Delay((uint32_t)50);
      NVIC_SystemReset();
    } 
			
  }//while(1)
  
}
//--------------------------MAIN----------------------------------------------------------------//

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

//stm32f4xx_it.c-hard fault

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
