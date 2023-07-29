/**
  ******************************************************************************
  * @file    Audio.c 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    8-Nov-2016
  ******************************************************************************
  */ 

//----------------------------------------------------------------------------//

#include "Audio.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include <stdlib.h>
//----------------------------------------------------------------------------//
uint8_t IsSoundConfigured;//сконфигурировано ли железо

float SoundGain=1.5f;//усиление(возможны искажения при переполнении ШИМ)
static uint16_t BeepLen=0;//длина мелодии в тонах,с паузами
static uint16_t* BeepArray;//указатель массива мелодии
volatile uint32_t TIM4_Cycles=0;//циклов TIM5 повторять при генерации тона
Aud_ty_cb* Audio_cb=0;//callback for audio timer

/*
генерация тона синусом приближенно:
  0)центр->
  1)центр+центр/2->
  2)макс->
  3)центр+центр/2->
  4)центр->
  5)центр-центр/2->
  6)мин->
  7)центр-центр/2
  --8 этапов всего--
*/
#pragma data_alignment = 4
static const float fSinePseudo[8] ={0.0f,0.5f,1.0f,0.5f,0.0f,-0.5f,-1.0f,-0.5f};

static int16_t* volatile NextBufferSamples;
static volatile int NextBufferLength;
static volatile int BufferNumber;
static volatile bool DMARunning;
HMP3Decoder hMP3Decoder;
bool disableAudio=false;
//----------------------------------------------------------------------------//
//MPEG1 Layer3 = 1152 x channels samples
//пример:
//2 канала 2 фрейма -минимально необходимый размер буфера должен составлять 2304*2=4608 байт:
//1 фрейм=2304 2=4608 3=6912 4=9216 
#define AUD_IN_BUF_SIZE     6912 //4096  //входной буфер 16*512=оптимально (загрузка,в байтах)

//samples output buffers x 2
#define AUD_FRAME_MAX_SIZE_PER_CHANNEL  1152 //максимальный размер в семплах для одного канала
#define AUD_MAX_CHANNELS                2    //максимум каналов
#define AUD_MAX_OUT_FRAMES              3    //количество максимальных фреймов целиком в выходном PCM буфере
#define AUD_MAX_OUT_BUFFERS             2    //буферы следуют один за другим
//PCM - выход, 2 буфера (выход семплов,в словах)
#define AUD_OUT_BUF_SIZE    (AUD_FRAME_MAX_SIZE_PER_CHANNEL* \
                             AUD_MAX_CHANNELS* \
                             AUD_MAX_OUT_FRAMES ) //(2=18432)/(3 фрейма=27648)/(4 фрейма=36864) байт
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
static bool OutSamplesWithCallBack(short* buffer,void* context);
//----------------------------------------------------------------------------//
//использованеие инструкций монопольного доступа CPU (перерасход рантайма!)
#define FAST_SEMAPHORE 0 
/*
инструкции захвата семафора вывода
*/
#if FAST_SEMAPHORE==1
  static volatile unsigned long _semaphore_=0;
  #define _slock   ThreadSafeLockUnlock32((unsigned long*)(&_semaphore_),true)

  #define _sunlock ThreadSafeLockUnlock32((unsigned long*)(&_semaphore_),false)

  #define _swait  while(__LDREXW(&_semaphore_)!=0)__CLREX();__CLREX() //ожидание семафора

  #define _it_unlock Atom32Set((unsigned long*)(&_semaphore_),0)
#else
  static Event* pEvReady=0;//синхрособытие готовности подгрузки PCM
#endif
//----------------------------------------------------------------------------//
//чтение из файла [с предварительным смещением,если не 0]
FRESULT ReadFromFileWithAbsOffset(FIL* fp,uint8_t* outBuffer,UINT abs_offset,UINT* rd_bytes)
{
  FRESULT fres;
  if(abs_offset)
  {
    if(abs_offset>=f_size(fp))return FR_EOF; 
    if((fres=f_lseek(fp,abs_offset))!=FR_OK)//смещение на начало данных
    return fres;//FS error
  }
  if(f_eof(fp))return FR_EOF;
  fres=f_read(fp,outBuffer,f_size(fp)-fp->fptr>=AUD_IN_BUF_SIZE?AUD_IN_BUF_SIZE:f_size(fp)-fp->fptr,rd_bytes);
  return fres;
}
//----------------------------------------------------------------------------//
//воспроизвести файл (предыдущий будет остановлен)
FRESULT PlayMP3File(TCHAR* filename)
{
  FRESULT fr=FR_OK;
  FIL* pf=NULL;
  MP3FrameInfo mp3FrameInfo;//=ccmalloc(sizeof(MP3FrameInfo));
  if(hMP3Decoder)MP3FreeDecoder(hMP3Decoder);
  hMP3Decoder=MP3InitDecoder();
  uint8_t* audioData=0;
  short* audioPCM_Out=0;
  //----------------------------------
  int offset=0;
  int err=0;
  int stot=0;//всего семплов в буфере
  UINT rdb=0;//bytes readed
  
  int outOfData=0;//обработка аудиопотока mp3 завершена
  const char *read_ptr=0;
  int bytes_left=0;//данных во входном буфере
  int bufnum=0;//номер текущего буфера
  UINT pMem=0; //file pointer (rd)
  int sampst=0;//total samples now in PCM buffer for output
  //----------------------------------
#if FAST_SEMAPHORE==1
    _swait;//ожидать семафора вывода
#else 
  if(pEvReady)
  {
    WaitForSingleObject((HANDLE)pEvReady,INFINITE);//ожидать готовности 
    SetEvent((HANDLE)pEvReady);
  }
  else pEvReady=(Event*)CreateEvent(FALSE,TRUE);// тип сброса=авто
  //if(pEvReady==0){while(1);return FR_INT_ERR;}
  
#endif
    
  
  if(filename)//новый файл
  {
    pf=mmmalloc(sizeof(FIL));//MMRAM!!!
    if(!pf)return FR_NO_FILE; //ERROR MEMORY ALLOCATION
    fr=f_open(pf,filename,FA_READ);
    if(fr!=FR_OK)
    {
      goto free_decoder_and_ret_fr;
    }//FS error
  }
  //----------------------------------
    
  audioData=mmmalloc(AUD_IN_BUF_SIZE);
  audioPCM_Out=mmmalloc(AUD_OUT_BUF_SIZE*AUD_MAX_OUT_BUFFERS*sizeof(short));
  if(audioPCM_Out==NULL||hMP3Decoder==NULL||audioData==NULL)//||mp3FrameInfo==NULL)
    goto free_decoder_and_ret_fr;// error
  //--------------------------------------------------------
  WHITE_LED_ON();GREEN_LED_OFF(); 
  //----------------------------
load_next_data:  //загрузка порции аудиопотока MP3
  pMem-=bytes_left;
  if((fr=ReadFromFileWithAbsOffset(pf,audioData,pMem,&rdb))!=FR_OK && fr!=FR_EOF)
  {
    //int n=0;
    goto free_decoder_and_ret_fr;// error
  }
  
  if(fr==FR_EOF)
  {
#if FAST_SEMAPHORE==1  
    _swait;
#else
    WaitForSingleObject((HANDLE)pEvReady,200);//INFINITE);//ожидать готовности
#endif    
    goto free_decoder_and_ret_fr;// error
  }
  
  bytes_left=(int)rdb;
  pMem+=rdb;
  //-------------------------
  //поиск начала фрейма
  read_ptr = (const char *)audioData;
    
find_begin_of_frame:
  offset = MP3FindSyncWord((unsigned char*)read_ptr,bytes_left);
  if(disableAudio || hMP3Decoder==0 || offset==-1)//фрейм не обнаружен
  {
    fr=FR_EOF;
    goto free_decoder_and_ret_fr;
  }
	
  read_ptr+=offset;
  bytes_left-=offset;
  mp3FrameInfo.bitrate=0;
  MP3GetNextFrameInfo(hMP3Decoder, &mp3FrameInfo, (unsigned char*)read_ptr);//как декодировать
  if(mp3FrameInfo.bitrate==0)
  {
    read_ptr+=256;
    bytes_left-=256;
    goto find_begin_of_frame;
  }
  
check_buff:
  //в какой буфер складывать семплы (чередовать для оптимального использования)
  if(stot>=AUD_OUT_BUF_SIZE||stot+mp3FrameInfo.outputSamps>AUD_OUT_BUF_SIZE ||
     ((outOfData||fr==FR_EOF)&&stot))
  {
    sampst=mp3FrameInfo.outputSamps;
    mp3FrameInfo.outputSamps=stot;
    stot=0;
    
    //--------вывод звука--------------->>
#if FAST_SEMAPHORE==1  
    _slock;
#else
    WaitForSingleObject((HANDLE)pEvReady,200);//ожидать готовности к выводу
#endif    
    
    //while(OutSamplesWithCallBack(audioPCM_Out+(AUD_OUT_BUF_SIZE*bufnum),&mp3FrameInfo)==false);
    //{fr=FR_INT_ERR;goto free_decoder_and_ret_fr;}
    //EnterCriticalSection;
    
    if(OutSamplesWithCallBack(audioPCM_Out+(AUD_OUT_BUF_SIZE*bufnum),(void*)&mp3FrameInfo)==false)
    {
      fr=FR_INT_ERR;
      //LeaveCriticalSection;
      goto free_decoder_and_ret_fr;
    }
    //continue decode
    //-------------------------
      if((outOfData||fr==FR_EOF)&&(fr=f_lseek(pf,0))!=FR_OK)
      {
        f_close(pf);
        //LeaveCriticalSection;
        goto free_decoder_and_ret_fr;// error
      }//смещение на начало данных
   
    
    mp3FrameInfo.outputSamps=sampst;
    bufnum++;
    if(bufnum>=AUD_MAX_OUT_BUFFERS)bufnum=0;
    WHITE_LED_TOGGLE();GREEN_LED_TOGGLE();
    
    //LeaveCriticalSection;
  }
  
  if(outOfData||fr==FR_EOF)
  {
    goto free_decoder_and_ret_fr;
  }
   
 /*if(ax.mp3FrameInfo.layer<1 || ax.mp3FrameInfo.layer>3)
  {
    ax.fr=FR_INVALID_PARAMETER;//не соответствует
    goto free_decoder_and_ret_fr;
  }*/
  //-----------------------
 //декодирование
 EnterCriticalSection;
 
 err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, &bytes_left,
                  audioPCM_Out+(AUD_OUT_BUF_SIZE*bufnum)+stot, 0);
  
 stot+=mp3FrameInfo.outputSamps;
  
 LeaveCriticalSection;
    
 if (err)
  {
    /* error occurred */
    switch (err) 
    {
     case ERR_MP3_INDATA_UNDERFLOW://данные закончились
	outOfData = 1;
        goto check_buff;
	
     case ERR_MP3_MAINDATA_UNDERFLOW:
      /* do nothing - next call to decode will provide more mainData */
	break;
     case ERR_MP3_FREE_BITRATE_SYNC:
     default:
        outOfData = 1;
        goto check_buff;
    }
  } 
  else
  {
    /* no error */
    MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
  }

  if(fr!=FR_EOF && fr!=FR_OK){goto free_decoder_and_ret_fr;}
  if(bytes_left<AUD_IN_BUF_SIZE/3)goto load_next_data;//подгрузить
    else goto find_begin_of_frame;//поиск следующего
  
free_decoder_and_ret_fr:
  MP3FreeDecoder(hMP3Decoder);hMP3Decoder=0;
  WHITE_LED_OFF();GREEN_LED_OFF(); 
  //---------------------if(ptask!=NULL)DeleteSystemTask(&ptask);
  FRESULT ffr=fr;
#if FAST_SEMAPHORE==1
  _swait;
#else
  //WaitForSingleObject((HANDLE)pEvReady,INFINITE);//ожидать готовности 
  SetEvent((HANDLE)pEvReady);
#endif
    
  msafe_free(audioData);
  msafe_free(audioPCM_Out);
  //msafe_free(mp3FrameInfo);
  //if(pEvReady){CloseHandle((HANDLE)pEvReady);msafe_free(pEvReady);}
  msafe_free(pf);
  return ffr;
}
//----------------------------------------------------------------------------//
/*
настройка и вывод семплов через ШИМ аудио
uint16_t* buffer-указатель на буфер семплов,
context-инфо фрейма
pEvReady-событие готовности
вернет false,если обработки не было
*/
static bool OutSamplesWithCallBack(short* buffer,void* context)
{
  static short* buff=0;
  static MP3FrameInfo mp3FrameInf;
  static int smprate=0;
  static volatile int samp=0;
  static volatile bool new_data=false;
  volatile bool last_sample=false;
  static float val;
  
  //BOOL rse;
  
  if(/*(pEvReady==NULL)||*/ (buffer==0 && ((mp3FrameInf.outputSamps==0)||
    (samp>=mp3FrameInf.outputSamps||samp>AUD_OUT_BUF_SIZE))))return false;
  
  
  if(buffer)
  {
    new_data=true;
    
    while(mp3FrameInf.outputSamps && Audio_cb && !last_sample && buff)
    {
      return false;//занято в данный момент
      //__asm__ volatile ("wfi"); //wait for interrupt
    }
    
    buff=buffer;mp3FrameInf=*((MP3FrameInfo*)context);
    
    samp=0;
    
    if(smprate!=mp3FrameInf.samprate || !Audio_cb)
    {
      if(!Audio_cb)Audio_cb=(Aud_ty_cb*)OutSamplesWithCallBack;
      TIMxLoadTimerAndStart(TIM4,1,(uint16_t)(84000000/mp3FrameInf.samprate),TIM_CR1_URS);
    }
    new_data=false;
    return true;
  }
  else if(buff==0)
  {
    Audio_cb=0;TIMxStopAndDisableIT(TIM4);
  }//отключить-буфера нет
  
  if(last_sample)
  {
    last_sample=false;
    goto set_audio_pwm;
  }
  //-------------output-------------
  samp+=mp3FrameInf.nChans;//соразмерно битам на канал
    
  val=(float)buff[samp]/32768.0f;
  //----------------------------
set_audio_pwm:
  //set audio pin output
  if(IsSoundConfigured)
  {
    float sv=val*SoundGain*(float)(SOUND_PWM_CENTER)+(float)(SOUND_PWM_CENTER);
    if(sv>(float)SOUND_PWM_LEN)sv=(float)SOUND_PWM_LEN;else if(sv<0.0f)sv=0.0f;
    TIM5->CCR4=(uint16_t)sv;//в регистре перезагрузки период ШИМ
    //TIM5->CCR4=(uint16_t)v;
  }
  //----------------------------
  if(samp>=mp3FrameInf.outputSamps)
  {
   if(!new_data)
    {
      Audio_cb=0;
      TIMxStopAndDisableIT(TIM4);
      smprate=0;
    }
  }
  else //обнулить на последнем для предварительной загрузки параметров прерывания
    if(samp+mp3FrameInf.nChans>=mp3FrameInf.outputSamps)
    {
      last_sample=true;
      val=(float)buff[samp]/32768.0f;
      buff=0;
      samp+=mp3FrameInf.nChans;
      mp3FrameInf.outputSamps=0;
#if FAST_SEMAPHORE==1
      _it_unlock;
#else
     if(pEvReady)SetEvent((HANDLE)pEvReady);
#endif
     if(disableAudio){Audio_cb=0;TIMxStopAndDisableIT(TIM4);}
    }
  
  return true;
}
//----------------------------------------------------------------------------//
void Beep(uint16_t tone,uint16_t timeMs)//вывести тон длительностью timeMs миллисекунд
{
  /* 
  точная формула расчёта частоты звучания ноты.
  если известен интервал в полутонах (n) от ноты До 1 октавы то любая нота может иметь частоту 
  F=261*2^(n/12)
  
  генерация тона синусом приближенно:
  0)центр->
  1)центр+центр/2->
  2)макс->
  3)центр+центр/2->
  4)центр->
  5)центр-центр/2->
  6)мин->
  7)центр-центр/2
  --8 этапов всего--
  */
  //запрещено менять частоту семплирования во время проирывания аудио. 
  if(!IsSoundConfigured || timeMs==0 || Audio_cb!=0)return;
  //TIMxStopAndDisableIT(TIM4);
  uint16_t sPrescaler=1;
  uint32_t sPeriod=(uint32_t)42000;
  
  
  if(tone==0)//pause
  {
    TIM4_Cycles=0;//только для паузы
    if(timeMs>0)
    {
      sPrescaler= (uint16_t)(timeMs*2);//84 MHz : ((prescaler-1)+1)*42000 CLK
      TIMxOnePulseAfterPeriod(TIM4,sPrescaler,(uint16_t)sPeriod);
    }
  }
  else
  {
    sPeriod=(uint32_t)(10500000/tone);//84 MHz/8 stages
    
    while(sPeriod>=65536)
    {
      sPrescaler++;
      sPeriod/=2;
    }
    
    TIM4_Cycles=(uint32_t)((uint32_t)timeMs*(uint32_t)tone*8/1000);
    TIMxLoadTimerAndStart(TIM4,sPrescaler,(uint16_t)sPeriod,TIM_CR1_URS);
  } 
}
//----------------------------------------------------------------------------//
void BeepMelody(uint16_t* MelodyArray,uint16_t notesTotal)
{
  BeepArray=MelodyArray;BeepLen=notesTotal;
  Beep(MelodyArray[0],MelodyArray[1]);
}
//----------------------------------------------------------------------------//
void BeepPause(uint16_t timeMs)//вывести звуковую паузу длительностью timeMks миллисекунд
{
  if(timeMs==0)return;else TIMxOnePulseAfterPeriod(TIM4,2*timeMs,42000);//ожидать
}
//----------------------------------------------------------------------------//
void TIM4_IRQHandler() //таймер длительности звука
{
  static uint8_t stage=0;//выборка
  static int32_t cycles=0;
  
  //TIM_CR1_URS=импульсный режим,таймер продолжит счет
  //TIM_CR1_OPM=режим одиночного импульса,таймер остановит счет автоматически
  
  TIM4->SR = (uint16_t)~TIM_SR_UIF;// Clear the IT pending Bit
  if(Audio_cb!=0){if(Audio_cb(0,0))return;}//приоритет аудиопотоку 
  
  if(cycles==0) //это был последний/первый цикл,отключить и проверить следующий тон
  {
    cycles=TIM4_Cycles;
    TIM4_Cycles=0;
       
    if(cycles==0)
    {
      TIMxStopAndDisableIT(TIM4);
      //next tone  
      if(BeepLen>0)
      {
       BeepLen--;
       if(BeepLen>0 && BeepArray!=0)
       {
         BeepArray+=2;Beep(BeepArray[0],BeepArray[1]);//повторная инициализация
       }
      }
    }
    
    return;
  }
  //нет новых данных,цикл семлирования продолжается  
  SetSoundVal(fSinePseudo[stage]);
  stage++;if(stage==8)stage=0;
  /*if(cycles>(int32_t)0)*/cycles--;
  //else TIMxStopAndDisableIT(TIM4);
  if(cycles==1)
  {
    cycles=1;
  }
}
//----------------------------------------------------------------------------//
void Sound_Config()  //конфигурация звука , PA3,TIM5 CH4 PWM
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP ;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM5);//PWM
  //-----настойка таймера,CLK=42*2=84 MHz-------
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
  /* Timer configuration in PWM mode */
  TIM_DeInit(TIM5);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  
  TIM_TimeBaseStruct.TIM_Prescaler=0; //CLK 84 MHz
  TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Down;  //считаем вниз 
  TIM_TimeBaseStruct.TIM_Period=(uint16_t)SOUND_PWM_LEN;//84Mhz/875 = 96 000 Hz выборка (MAX);437=center -период ШИМ
  TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStruct);
  TIM_UpdateRequestConfig(TIM5,TIM_UpdateSource_Regular);
  
  TIM_OCInitTypeDef oc_init;
  TIM_OCStructInit(&oc_init);
  oc_init.TIM_OCMode = TIM_OCMode_PWM1;//режим1 front align(TIM_OCMode_PWM2-по центру)
  oc_init.TIM_OutputState = TIM_OutputState_Enable;//подключаем к выходу
  oc_init.TIM_OCPolarity = TIM_OCPolarity_Low;  // положительная полярность(3V=импульс)
  
  oc_init.TIM_Pulse = (uint16_t)(SOUND_PWM_LEN/2); //начальное заполнение
  TIM_OC4Init(TIM5,&oc_init);   // заносим данные в 4 канал - порт A3
  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
  
  TIM_Cmd(TIM5,ENABLE);   // запускаем счёт
  //------настройка таймера длительности TIM4 CLK=84 MHz---------
  Timer_Config(1,RCC_APB1Periph_TIM4,
               TIM4,
               1,42000, //СLK to 42 MHz(prescaler=1)
               TIM_CounterMode_Up,
               TIM4_IRQn,
               true);//false); //not starting
  
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//<=FS priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  IsSoundConfigured=1;
}
//----------------------------------------------------------------------------//
void SetSoundVal(float val)//задать текущую амплитуду выборки 0.0f- +/-1.0f от громкости
{
  if(!IsSoundConfigured)return;
  float cntrPWM=(float)SOUND_PWM_LEN/2.0f;
  float sv=val*SoundGain*cntrPWM+cntrPWM;
  if(sv>(float)SOUND_PWM_LEN)sv=(float)SOUND_PWM_LEN;else if(sv<0.0f)sv=0.0f;
  TIM5->CCR4=(uint16_t)sv;//в регистре перезагрузки период ШИМ
}
//----------------------------------------------------------------------------//
//                          WAV PLAYER

/******************* (C) COPYRIGHT 2016 dem1305 *****END OF FILE****/
