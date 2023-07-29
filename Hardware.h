/**
  Laser project by dem1305

  LScan hardware default settings of pins and ports

  @11/6/2015
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARE_H
#define __HARDWARE_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
/* ---------------------------------------------------------------------------*/
//#define DISCOVERY_BOARD //if debug in discovery dev.board
//#define TST_STREAM
/* ---------------------------------------------------------------------------*/
/* sequence:
#define x_PIN                GPIO_Pin_x
#define x_PORT               GPIOx
#define x_CLK                RCC_AHBxPeriph_GPIOx
#define x_EXTI_LINE          EXTI_Linex
#define x_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOx
#define x_EXTI_PIN_SOURCE    EXTI_PinSourcex
#define x_EXTI_IRQn          EXTIx_IRQn 
*/
/* ---------------------------------------------------------------------------*/
//ШИМ гашения лучей, digital out x3 
#define PWM_COLORS_TIMER            TIM1
#define PWM_COLORS_TIMER_CLK        RCC_APB2Periph_TIM1            
#define PWM_COLORS_PORT_SOURCE      GPIOE
#define PWM_COLORS_R_PIN_SOURCE     GPIO_Pin_9
#define PWM_COLORS_G_PIN_SOURCE     GPIO_Pin_11
#define PWM_COLORS_B_PIN_SOURCE     GPIO_Pin_13 
#define PWM_COLORS_AF_GPIO_SOURCE   GPIO_AF_TIM1
/* ---------------------------------------------------------------------------*/
//WLAN ESP32 (SPI1)
/* 
PB11 - HANDSHAKE[2](ACTIVE-HIGH)
PB5  - MOSI[23]
PB4  - MISO[19]
PB3  - SCK[18]
PE7  - SOFT_CS(5)
*/
/* ---------------------------------------------------------------------------*/
//PA10-power pin(HIGH=ON)
/* ---------------------------------------------------------------------------*/
//DAC X,Y positions analog out ports
#define DAC_X_Port        GPIOA
#define DAC_X_Pin         GPIO_Pin_4
#define DAC_Y_Port        GPIOA
#define DAC_Y_Pin         GPIO_Pin_5
#define DAC_CLK           RCC_AHB1Periph_GPIOA 
/* ---------------------------------------------------------------------------*/
//SDIO flash , MMC card
/*
D1=PC9
D0=PC8
CLK=PC12
DET=PE6
CMD=PD2
D3=PC11
D2=PC10
*/

//SPI flash , MMC card
/*
CS=PB12
CLK=PB10
MISO=PB14
MOSI=PB15
*/

//DPSS laser temperature control ADC 
#define ADC_G_temp_Port        GPIOC
#define ADC_G_temp_Pin         GPIO_Pin_1
#define ADC_G_reg_channel      11

//SPI3-не используется=WLAN UART 20MHz interface SCK, MOSI, MISO PB3 PB5 PB4 
//SPI1 (используется для WLAN SPI):
//GPIOB, 3//CLK
//GPIOB, 4//MISO TX
//GPIOB, 5//MOSI RX
//GPIOB, 8//HANDSHAKE
/* ---------------------------------------------------------------------------*/
/*
TIM 1,8,9,10,11 -CLK=168

задействованные таймеры:
SysTick - основной проигрывания
-----------------------------------------
TIM1 - шим цветов

-TIM6 - такты 1 мс векторного вывода + коррекция скорости

-TIM8 - PWM CH_2 для PID-GREEN modulation

TIM9,-10,11 - сдвиги цветов RBG

TIM4,TIM5 - Звук(ШИМ тона,ШИМ амплитуды)

TIM3 - ШИМ окна фронтальной панели(PC6),ШИМ sys_fan,green fan(PB0,PB1)

TIM7 - spi_flash,10 ms (spi_card.c)

-----------------------------------------
ПРИОРИТЕТЫ ПРЕРЫВАНИЙ : группа,подгруппа:
CORE_NVIC_PriorityGroup=NVIC_PriorityGroup_2 //[0-3] приоритет:[0-3]подприоритет

OS CORE = 3,0-2
USB OTG_FS_IRQn = 1,3

SD_DMA =1,0
SD_SDIO=0,0

PLAY_TIMER_IT_PRIO            2,2
RGB_OFFSET_TIM_IT_PRIO        2,1
WLAN_UART2_IT_PRIO            1,0
WLAN_UART2_DMA_RX_IT_PRIO     1,1
WLAN_UART2_DMA_TX_IT_PRIO     1,1

ADC1 температурного контроля  3,3
ADC1 DMA2_Stream0             3,3
-----------------------------------------
USER FRONTAL PANEL BUTTONS: PA8, PD9-PD15 
*/
/* ---------------------------------------------------------------------------*/

//ADC1 pins 
//PA1=channel 1 PC1=channel 11
//ADC2 pins 
//PA2=channel 2 PC2=channel 12
/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum 
{
  LEDRED = 0,
  LEDGREEN = 1,
  LEDWHITE = 2,
  LEDORANGE= 2, //discovery
  LEDBLUE = 3
} Led_TypeDef;


/*
typedef enum 
{  
  BUTTON_USER = 0,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;     
*/



#ifdef DISCOVERY_BOARD
/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_LED
  * @{
  */
  //green-PD12
  //orange-PD13
  //red-PD14
  //blue-PD15
  #define LEDn                               4
  
  #define LEDRED_PIN                         GPIO_Pin_14
  #define LEDRED_GPIO_PORT                   GPIOD
  #define LEDRED_GPIO_CLK                    RCC_AHB1Periph_GPIOD  
    
  #define LEDGREEN_PIN                       GPIO_Pin_12
  #define LEDGREEN_GPIO_PORT                 GPIOD
  #define LEDGREEN_GPIO_CLK                  RCC_AHB1Periph_GPIOD  
    
  #define LEDBLUE_PIN                        GPIO_Pin_15
  #define LEDBLUE_GPIO_PORT                  GPIOD
  #define LEDBLUE_GPIO_CLK                   RCC_AHB1Periph_GPIOD 
  
  #define LEDORANGE_PIN                      GPIO_Pin_13
  #define LEDORANGE_GPIO_PORT                GPIOD
  #define LEDORANGE_GPIO_CLK                 RCC_AHB1Periph_GPIOD
   //adopt 
  #define LEDWHITE_PIN                       LEDORANGE_PIN
  #define LEDWHITE_GPIO_PORT                 LEDORANGE_GPIO_PORT
  #define LEDWHITE_GPIO_CLK                  LEDORANGE_GPIO_CLK
  /**
    * @}
    */ 
    
  /** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_BUTTON
    * @{
    */  
  #define BUTTONn                          1  
  
  /**
   * @brief Wakeup push-button
   */
  #define USER_BUTTON_PIN                GPIO_Pin_0
  #define USER_BUTTON_GPIO_PORT          GPIOA
  #define USER_BUTTON_GPIO_CLK           RCC_AHB1Periph_GPIOA
  #define USER_BUTTON_EXTI_LINE          EXTI_Line0
  #define USER_BUTTON_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOA
  #define USER_BUTTON_EXTI_PIN_SOURCE    EXTI_PinSource0
  #define USER_BUTTON_EXTI_IRQn          EXTI0_IRQn 
  /**
    * @}
    */ 
#else //LScan2 board

  #define LEDn                             4
  
  #define LEDRED_PIN                         GPIO_Pin_2
  #define LEDRED_GPIO_PORT                   GPIOE
  #define LEDRED_GPIO_CLK                    RCC_AHB1Periph_GPIOE  
    
  #define LEDGREEN_PIN                       GPIO_Pin_3
  #define LEDGREEN_GPIO_PORT                 GPIOE
  #define LEDGREEN_GPIO_CLK                  RCC_AHB1Periph_GPIOE  
    
  #define LEDWHITE_PIN                       GPIO_Pin_0
  #define LEDWHITE_GPIO_PORT                 GPIOD
  #define LEDWHITE_GPIO_CLK                  RCC_AHB1Periph_GPIOD  
    
  #define LEDBLUE_PIN                        GPIO_Pin_9
  #define LEDBLUE_GPIO_PORT                  GPIOA
  #define LEDBLUE_GPIO_CLK                   RCC_AHB1Periph_GPIOA
  //adopt 
  #define LEDORANGE_PIN                      LEDWHITE_PIN
  #define LEDORANGE_GPIO_PORT                LEDWHITE_GPIO_PORT
  #define LEDORANGE_GPIO_CLK                 LEDWHITE_GPIO_CLK
#endif

void LSCAN2_LEDInit(Led_TypeDef Led);
void LSCAN2_LEDOn(Led_TypeDef Led);
void LSCAN2_LEDOff(Led_TypeDef Led);
void LSCAN2_LEDToggle(Led_TypeDef Led);

//SOUND (PWM output)
#define SOUND_PIN                         GPIO_Pin_3
#define SOUND_GPIO_PORT                   GPIOA
#define SOUND_GPIO_CLK                    RCC_AHB1Periph_GPIOA  
#define SOUND_GPIO_PINSOURCE              GPIO_PinSource3
//------------FAN's-----------------------------------
//ADC
#define SFAN_ADC                          ADC1
#define SFAN_ADC_CHANNEL                  7
#define SFAN_ADC_CLK                      RCC_APB2Periph_ADC1
#define SFAN_ADC_PIN                      GPIO_Pin_7
#define SFAN_ADC_GPIO_PORT                GPIOA
#define SFAN_ADC_GPIO_CLK                 RCC_AHB1Periph_GPIOA
//PWM System FAN (general cooling of optics) 
#define SFAN_PWM_PIN                      GPIO_Pin_1
#define SFAND_PWM_GPIO_PORT               GPIOB
#define SFAN_PWM_GPIO_CLK                 RCC_AHB1Periph_GPIOB
#define SFAN_PWM_GPIO_PINSOURCE           GPIO_PinSource1

//------------------------------------------------------------------------------
/*
сколько данных в счетчике передач
вход-DMAy_Streamx поток,
(размерность данных в байтах:DMA_xxx_Byte  = 1 DMA_xxx_HalfWord  =2  DMA_xxx_Word = 4)     
возврат количества оставшихся элементов(макс 65535)
*/
inline uint32_t getDMA_counter(DMA_Stream_TypeDef* DMAy_Streamx)
{
  return ((uint32_t)0xFFFF)- DMAy_Streamx->NDTR;
}
/* ---------------------------------------------------------------------------*/
//гду точка разделения
inline uint8_t prio_pt_pos(uint32_t NVIC_PriorityGroup)
{
 //PRIGROUP: поле приоритетной группировки прерываний
 //Это поле определяет разделение группового приоритета от подприоритета,
 //см. Двоичную точку на стр. 227 programming manual.
  switch(NVIC_PriorityGroup)
  {
    case NVIC_PriorityGroup_0: //0 bits for pre-emption priority
                               //   4 bits for subpriority
      return 4;
      break;
    case NVIC_PriorityGroup_1: //1 bits for pre-emption priority
                               //   3 bits for subpriority
      return 3;
      break;
    case NVIC_PriorityGroup_2: //2 bits for pre-emption priority
                               //   2 bits for subpriority
      return 2;
      break;
    case NVIC_PriorityGroup_3: //3 bits for pre-emption priority
                               //   1 bits for subpriority
      return 1;
      break;
    case NVIC_PriorityGroup_4: //4 bits for pre-emption priority
                               //   0 bits for subpriority
      return 0;
      break;
  default:while(1);
  }
}
//----------------------------------------------------------------------------//
inline void NVIC_set_group_prio(IRQn_Type IRQn, uint32_t priority,uint32_t subpriority)
{
  uint32_t NVIC_PriorityGroup=SCB->AIRCR & 0x700;//получаем группировку(со сдвигом)
  NVIC_SetPriority(IRQn, ((priority<<prio_pt_pos(NVIC_PriorityGroup))|subpriority));//на сколько сдвинуть значение приоритета
}
//----------------------------------------------------------------------------//
/*
получить приоритет прерывания согласно группировке
(позиции точки приоритет.подприоритет) приоритетов.
если pstore не 0, то сохранить кодированное значение по указателю.
*/
inline uint32_t GetITPrioLevelAndStoreEncoded(IRQn_Type IRQn,uint32_t PriorityGroup,uint32_t* pstore)
{
  uint32_t tencoded=NVIC_GetPriority(IRQn);//получение абсолютного (кодированного) значения
  if(pstore)*pstore=tencoded;
  uint32_t prio,sub;//переменные для сохранения
  //декодировать (разбить на приоритет/подприоритет) согласно группировке (позиции точки)
  NVIC_DecodePriority (tencoded, PriorityGroup, &prio, &sub);
  return prio;
}
//----------------------------------------------------------------------------//

#endif /* __HARDWARE_H */
/******************* (C) COPYRIGHT 2015 dem1305 *****END OF FILE****/
