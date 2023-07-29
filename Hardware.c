/**
  ******************************************************************************
  * @author  dem1305
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Hardware.h"
//------------------------------------------------------------------------------
GPIO_TypeDef* GPIO_PORT[LEDn] = {LEDRED_GPIO_PORT, LEDGREEN_GPIO_PORT, LEDWHITE_GPIO_PORT,
                                 LEDBLUE_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LEDRED_PIN, LEDGREEN_PIN, LEDWHITE_PIN,
                                 LEDBLUE_PIN};
const uint32_t GPIO_CLK[LEDn] = {LEDRED_GPIO_CLK, LEDGREEN_GPIO_CLK, LEDWHITE_GPIO_CLK,
                                 LEDBLUE_GPIO_CLK};

void LSCAN2_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(GPIO_CLK[Led], ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

void LSCAN2_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRL = GPIO_PIN[Led];
}

void LSCAN2_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRH = GPIO_PIN[Led];  
}

void LSCAN2_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/******************* (C) MW dem1305 *****END OF FILE****/
