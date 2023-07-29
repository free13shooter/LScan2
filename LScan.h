/**
  Laser project by dem1305

  @2014
*/
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
#ifndef __LSCAN_H
#define __LSCAN_H

//----------------------------------------------------------------------------//
#include "Hardware.h"
#include "main.h"
#include <stdlib.h>
#include <string.h> //mem... in DLib_Product
#include "math.h"
//#include "arm_math.h"        //PID,basic math func's
#include "stm32f4xx_rng.h"     //случайный генератор
//#include "usbd_custom_core.h"  //USB defines
//===========
#include "CCM_MM_RAM.h"    //возможность использования 64 КБ памяти Core-Couped-Memory для дополнительной кучи
//#include "sys_timers.h" //системные таймеры,выполняемые в контексте прерываний systick
//#include "sys_tasks.h"  //отложенные системные задачи,выполняемые в контексте основного потока
#include "core.h"
//===========
//#include "UART.h"
//===========
#include "ff.h"//file system (FAT32)
//===========
#include "font_func.h"  //управление выводом векторного текста
//===========
#include "Audio.h" //mp3/wav
//===========

//#define SMALL_STREAM_BUFFER
//#define SMALLEST_STREAM_BUFFER

#if defined(SMALL_STREAM_BUFFER)
  #define STREAMBUF_SIZE          8192
  #define STREAMBUF_SIZE_1_2      4096                    // 1/2
  #define STREAMBUF_SIZE_DIV2     STREAMBUF_SIZE_1_2
  #define STREAMBUF_SIZE_3_4      6144                    // 3/4 of buff size
  #define SPI_DMA_BLOCK_SIZE      512  
  #define SPI_DMA_BLOCKx2         1024 
#elif defined(SMALLEST_STREAM_BUFFER)
  #define STREAMBUF_SIZE          768
  #define STREAMBUF_SIZE_1_2      384                     // 1/2
  #define STREAMBUF_SIZE_DIV2     STREAMBUF_SIZE_1_2
  #define STREAMBUF_SIZE_3_4      576                     // 3/4 of buff size
  #define SPI_DMA_BLOCK_SIZE      256  
  #define SPI_DMA_BLOCKx2         512 
#else
  #define STREAMBUF_SIZE          10240
  #define STREAMBUF_SIZE_1_2      5120                    // 1/2
  #define STREAMBUF_SIZE_DIV2     STREAMBUF_SIZE_1_2
  #define STREAMBUF_SIZE_3_4      7680                    // 3/4 of buff size
  #define SPI_DMA_BLOCK_SIZE      1024                    // кратно буферу
  #define SPI_DMA_BLOCKx2         2048  
#endif

extern uint8_t StreamBuf[];//буфер данных воспроизведения FIFO.
extern uint8_t* StreamOverbuf;
extern volatile uint8_t* wrPtr;
extern volatile uint8_t* rdPtr;
extern volatile uint8_t  PlayFlag;//флаг воспроизведения
extern bool FullSysReset;
extern uint8_t LScanStatus;
extern volatile uint32_t rdy_ilda; 
//----------------------------------------------------------------------------//
//ERRORS
#define ERR_DATA_ZERO           1 //нули в потоке
#define ERR_DATA_ZERO_PROCESSED 2 //обработвно
#define ERR_FREE_NEGATIVE       4 //св.место отриц.
//----------------------------------------------------------------------------//
#define GREEN_LED_OFF()       LSCAN2_LEDOff(LEDGREEN)//GREEN
#define WHITE_LED_OFF()       LSCAN2_LEDOff(LEDWHITE)//WHITE
#define ORANGE_LED_OFF()      LSCAN2_LEDOff(LEDORANGE)//ORANGE
#define RED_LED_OFF()         LSCAN2_LEDOff(LEDRED)//RED
#define BLUE_LED_OFF()        LSCAN2_LEDOff(LEDBLUE)//BLUE 

#define GREEN_LED_ON()        LSCAN2_LEDOn(LEDGREEN)//GREEN
#define WHITE_LED_ON()        LSCAN2_LEDOn(LEDWHITE)//WHITE
#define ORANGE_LED_ON()       LSCAN2_LEDOn(LEDORANGE)//WHITE
#define RED_LED_ON()          LSCAN2_LEDOn(LEDRED)//RED
#define BLUE_LED_ON()         LSCAN2_LEDOn(LEDBLUE)//BLUE 

#define GREEN_LED_TOGGLE()    LSCAN2_LEDToggle(LEDGREEN)//GREEN
#define WHITE_LED_TOGGLE()    LSCAN2_LEDToggle(LEDWHITE)//WHITE
#define ORANGE_LED_TOGGLE()   LSCAN2_LEDToggle(LEDORANGE)//ORANGE
#define RED_LED_TOGGLE()      LSCAN2_LEDToggle(LEDRED)//RED
#define BLUE_LED_TOGGLE()     LSCAN2_LEDToggle(LEDBLUE)//BLUE 
//----------------------------------------------------------------------------//
#define USE_CLR_SHIFTING
#define USE_SPD_CORRECTOR

#define DISCR_CONST_VALUE           20000
//предел миллисекундной дискретизации
#define LIM_DISCR_MS                (ISOC_OUT_PACKET_SIZE/sizeof(ICP))     
#define MAX_DISCR                   50000   //(ISOC_EP_PK_SIZE/sizeof(SPOINT)*1000) -не более
#define OPT_DISCR                   20000
#define MIN_DISCR                    3000   //менее 1000 не может быть
#define MIN_DISCR_MS                (MIN_DISCR/1000)
#define OPT_DISCR_MS                (OPT_DISCR/1000)
#define MAX_DISCR_MS             ((int)(MAX_DISCR/1000)) //постоянная максимальная величина дискретизации в мс

#define MAX_FRAMES_PER_SEC             25
#define MIN_FRAMES_PER_SEC             20

#define MAX_ISOPACKET             (MAX_DISCR/1000*sizeof(ICP))
#define MIN_ISOPACKET             (MIN_DISCR/1000*sizeof(ICP))  

#define LSCAN_SIZE_PROP_DIV       18  //65536/4096 -делитель размерности разрешения DAC STM32
//----------------------------------------------------------------------------//
// [0-3]:[0-3] приоритеты:подприоритеты групп прерываний
//--------------------------------------
#define USB_FS_IT_PRIO                1  //1
#define USB_FS_IT_PRIO_SUB            3  //3

#define PLAY_TIMER_IT_PRIO            1
#define PLAY_TIMER_IT_PRIO_SUB        3 

#define INTERVAL_TIMER_IT_PRIO        1
#define INTERVAL_TIMER_IT_PRIO_SUB    3

#define R_OFFSET_TIM_IT_PRIO          1
#define R_OFFSET_TIM_IT_PRIO_SUB      1

#define G_OFFSET_TIM_IT_PRIO          1
#define G_OFFSET_TIM_IT_PRIO_SUB      0

#define B_OFFSET_TIM_IT_PRIO          1
#define B_OFFSET_TIM_IT_PRIO_SUB      1
//--------------------------------------
#define WLAN_UART2_IT_PRIO            1
#define WLAN_UART2_DMA_RX_IT_PRIO     1
#define WLAN_UART2_DMA_TX_IT_PRIO     1

//--------------------------------------
#define SPI3_IT_PRIO                  0
#define SPI3_DMA_TX_IT_PRIO           0

#define SPI_TIMER_IT_PRIO             3
#define SPI_TIMER_IT_PRIO_SUB         0
//--------------------------------------
       
//----------------------------------------------------------------------------//
//учет скорости модуляции 
//#define DPSS_DISCR            15000 //оптимальная скорость модуляции зеленого лазера

//#define G_LASER_IS_DPSS       1   //тип зеленого лазера
//#define G_LASER_MAX_DISCR_MS  (DPSS_DISCR/1000)   //максимальная скорость модуляции
//#define G_LASER_fPERIOD       ((float)1.0f/(float)G_LASER_MAX_DISCR_MS)
//#define G_LASER_TRESHOLD      20  //проценты от макимальной мощности
//----------------------------------------------------------------------------//
//нумерация цветов
#define RED_LASER       1
#define GREEN_LASER     2
#define BLUE_LASER	3

#define RED_COLOR	RED_LASER
#define GREEN_COLOR     GREEN_LASER
#define BLUE_COLOR	BLUE_LASER

//размерность координатной шкалы относительно 12 бит 
#define  maxXCoordValue  4095  //[2048-центр-]
#define  minXCoordValue  0
#define  maxYCoordValue  4095
#define  minYCoordValue  0
//центры шкал DACs
#define centreX 2048
#define centreY 2048

//#define MONO_COLOR 
#define COLORS_PWM_FREQ_MHZ   168 //частота ШИМ модуляции цветов
#define COLORS_PWM_PERIOD     256 //ширина ШИМ цветов

#define REPEAT_CYCLES         20    //количество повторов семпла при отсутствии данных

//#define _setDACs(valX,valY)   DAC->DHR12R1=(valX);DAC->DHR12R2=(valY) //установить знацения ЦАП x,y
#define _setDACs(valX,valY)   {uint32_t data=((uint32_t)(valY) << 16);data |= (valX);(DAC->DHR12RD)=data;}
#define _setDACX(valX)   DAC->DHR12R1=(valX) //установить знацение ЦАП 
#define _setDACY(valY)   DAC->DHR12R2=(valY) //установить знацение ЦАП 

//#define _setRGB(r,g,b) {TIM1->CCR1=(uint16_t)(r);TIM1->CCR2=(uint16_t)(g);TIM1->CCR3=(uint16_t)(b); \
                        R=(uint8_t)(r);G=(uint8_t)(g);B=(uint8_t)(b);}

#define _setRGB(r,g,b) {TIM1->CCR1=(uint16_t)(R_PWM[(r)]); \
                        TIM1->CCR2=(uint16_t)(G_PWM[(g)]); \
                        TIM1->CCR3=(uint16_t)(B_PWM[(b)]); \
                        R=(uint8_t)(r);G=(uint8_t)(g);B=(uint8_t)(b);}

#define  _setR(r) {TIM1->CCR1=(uint16_t)(R_PWM[(r)]);R=(r);}
#define  _setG(g) {TIM1->CCR2=(uint16_t)(G_PWM[(g)]);G=(g);}
#define  _setB(b) {TIM1->CCR3=(uint16_t)(B_PWM[(b)]);B=(b);}

//----------------------------------------------------------------------------//
extern void HardFault_Handler(void);
//----------------------------------------------------------------------------//
//Инициализация
void LScan_Init(void);
void LScan_Denit(void);
uint32_t u32Rnd();//аппаратный генератор случайных чисел 32 бит 0-2^32 = 4,294,967,295 (0xFFFFFFFF==UINT32_MAX),LScan_types.h
float fRnd();//0-1.0f
#define bRnd (u32Rnd()>=HALF_UINT32_MAX?true:false) //boolean random
int iRnd(int min,int max); //int random от min до max включительно
//ШИМ- модуляторы гашения лучей
void Colors_PWM_Config(void);  //лазерные модуляторы (TIM13,TIM1,TIM14) 84mhz 168mhz 84mhz PA6 PE11 PA7
//ШИМ-контроль амплитуды гашения лучей
void ADC_Reg_Config(void);//ацп температурной коррекции ШИМ DPSS-GREEN лазера,12 бит вход PC1
void ADC_check_complete();
//Задание координат
void DAC_XY_Config(void);    //цап координат,2*12 бит выходы
//----------------------------------------------------------------------------//
void SetRGB(uint8_t r,uint8_t g,uint8_t b);//установка периодов шим
inline void SetR(uint8_t r);
inline void SetG(uint8_t g);
inline void SetB(uint8_t b);
//----------------------------------------------------------------------------//
void PID_Timer_Config(uint16_t TimeStep);//таймер запуска преобразований ADC
//----------------------------------------------------------------------------//
//настройка таймера
void Timer_Config(uint8_t APB_1_or_2,uint32_t RCC_APB_Periph,
                  TIM_TypeDef* TIMx,
                  uint16_t prescaler,uint16_t period,
                  uint16_t TIM_CounterMode,
                  uint8_t IRQ_Type,//255=нет прерываний
                  bool StartNow);//запуск?
//загрузка счетчика и запуск таймера
void TIMxOnePulseAfterPeriod(TIM_TypeDef* TIMx,uint16_t prescaler,uint16_t reload);
//загрузка счетчика и запуск таймера,с учетом режима
void TIMxLoadTimerAndStart(TIM_TypeDef* TIMx,uint16_t prescaler,uint16_t reload,uint16_t pulse_mode);
//остановка и отключение прерываний таймера
void TIMxStopAndDisableIT(TIM_TypeDef* TIMx);
void getTIM_par(uint16_t* pprescaler,uint16_t* pperiod,uint32_t delay);

//----------------------------------------------------------------------------//
void uDelay (const uint32_t usec);//microsec
void mDelay (const uint32_t msec);//millisec
//----------------------------------------------------------------------------//
//random generator
//void RNG_config();
//uint32_t GetRNG();//возврат 32 бита случайное число
//----------------------------------------------------------------------------//
void ReloadPWMColors();//перерасчет и загрузка ШИМ цветов
void Calculate_PWM_Table(int color);//рассчитать массив заполнений
void Calculate_Colors_Shifting_and_Sampling();//рассчитать таблицы смещений
//----------------------------------------------------------------------------//
void ButSensorsInit(); //инициализация кропок и датчиков крайних положений окна защиты
void FANs_Servo_PWM_config();//инициализация ШИМ вентиляторов и серво окна безопасности
//----------------------------------------------------------------------------//

#define max(a,b) ((a)>(b)?(a):(b))
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
void LED_PanicLoop(uint32_t numLoops);
inline void LED_Panic(){LED_PanicLoop(0);}
//----------------------------------------------------------------------------//
void PlayTimers_Config();
//----------------------------------------------------------------------------//
//включение/выключение канала прерывания core_cm4.h
//задача мониторинга аппаратного состояния
//void* Hardware_Monitoring_Task(void* tctx);
//deff_ty_task Hardware_Monitoring_Task;

//----------------------------------------------------------------------------//
//аппаратное копирование с помощью DMA2
void DMA2_memcpy(DMA_Stream_TypeDef* DMAy_Streamx,void * _D, const void * _S, size_t _N);

void SysTick_Handler(void);
//----------------------------------------------------------------------------//
#define OPEN_ANGLE      0
#define CLOSE_ANGLE     190
#define SERVO_OFF_CNTR  20  //макс.значение декрементального счетчика для main
//преобразовать угол в значение ШИМ
void _wnd_servo_angle(uint16_t angle);
void ServoPinConfig(bool on);
uint16_t ServoGradToPWM(uint16_t grad,SERVO_PAR* servoLimits);
void Wnd_OPEN();//открыть окно
void Wnd_CLOSE();//закрыть окно
//----------------------------------------------------------------------------//
//питание
void Power_pin_config();
void Power_ON();
void Power_OFF();
void Power_Toggle();
void Power_off_timer_func();//выключение при бездействии
//----------------------------------------------------------------------------//
//кнопки (8 штук)
int ButtonsScanFunc(void* pargs);
void Buttons_config();

//нажата ли кнопка
bool isButtonPressed(uint8_t butIndex);

//нажата ли кнопка,с учетом времени нажатия
bool isButtonPressedLong(uint8_t butIndex);//долгий нажим
bool isButtonPressedShort(uint8_t butIndex); //короткий нажим

//события (фиксация после отжатия).Обнуляются при опросе.
bool butEventClick(uint8_t butIndex);
bool butEventPress(uint8_t butIndex);

//ассоциации индекса кнопок
#define but_operate 7
#define but_stop    6
#define but_eject   6
#define but_pause   5
#define but_rec     4
#define but_rew     3
#define but_play    2
#define but_ff      1
#define but_pict    0
#define but_power   0

#define BUTTON_REP_MS    100  //время повторения опроса сканирования кнопок
#define BUTTONS_TCK      4    //0.5c антидребезг,в тактах таймера
#define BUTTONS_REP      3    //накопление нажатий при удерживании         
//----------------------------------------------------------------------------//
bool SD_detect_processing(void);//обработка прерывания 
//обслуживание SD,фоновая задача
int SD_CARD_processing(void* ctx);
//----------------------------------------------------------------------------//
void MEMS_DPM_MIC_Config(void);
//----------------------------------------------------------------------------//
/*
        LScan2 intercon WLAN  
	Password: inferno1
	TCP2UART connect: 192.168.13.2:2264, Web connect: 192.168.13.2:2265
*/
//----------------------------------------------------------------------------//
/*
воспроизвести файл с учетом громкости.
если громкость не задана,то остается как есть
*/
FRESULT Sound_notification(float volume,TCHAR* p_file_name);
//----------------------------------------------------------------------------//
void set_setup_center(uint8_t r,uint8_t g,uint8_t b);//для настройки сведения
//----------------------------------------------------------------------------//
#endif /* __LSCAN_H */
/******************* (C) COPYRIGHT 2015 dem1305 *****END OF FILE****/
