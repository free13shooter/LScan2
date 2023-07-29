/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
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
#include "stm32f4xx_it.h"
#include "main.h"
#include "Hardware.h"
#include "LScan.h"
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */

#define HFSR_DEBUG_VT_BIT 31
#define HFSR_FORCED_BIT   30
#define HFSR_VECTTBL_BIT  1

#define HFSR_DEBUG_VT (1UL<<HFSR_DEBUG_VT_BIT)
#define HFSR_FORCED   (1UL<<HFSR_FORCED_BIT)
#define HFSR_VECTTBL  (1UL<<HFSR_VECTTBL_BIT)
/*
void HardFault_Handler(void)
{
  // Go to infinite loop when Hard Fault exception occurs 
  uint32_t er=SCB->HFSR;
  
  uint8_t r=0;//0000[obgr] leds
  if(er& HFSR_DEBUG_VT)r|=1;  //red
  if(er& HFSR_FORCED)r|=2;    //green
  if(er& HFSR_VECTTBL)r|=4;   //blue
  
  while (1)
  {
    if(r&1) LSCAN2_LEDOn(LEDRED);
    if(r&2) LSCAN2_LEDOn(LEDGREEN);
    if(r&4) LSCAN2_LEDOn(LEDBLUE);
    
    it_mDelay(350);
    LSCAN2_LEDOff(LEDRED);
    LSCAN2_LEDOff(LEDGREEN);
    LSCAN2_LEDOff(LEDBLUE);
    LSCAN2_LEDOff(LEDORANGE);
    it_mDelay(350);
  }
}
*/
/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
/*
void MemManage_Handler(void)
{
  while (1)
  {
  }
}
*/
/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
#define BFARVALID   (1UL<<15)
#define LSPERR      (1UL<<13)
#define STKERR      (1UL<<12)
#define UNSTKERR    (1UL<<11)
#define IMPRECISERR (1UL<<10)
#define PRECISERR   (1UL<<9)
#define IBUSERR     (1UL<<8)
/*
void BusFault_Handler(void)
{
  uint32_t er=SCB->CFSR;//[15:8] 33280!
  
  uint32_t pau=650;
  
  uint8_t r=0;//0000[obgr] leds
  if(er& BFARVALID)r|=1;  //red
  if(er& LSPERR)r|=2;    //green
  if(er& STKERR)r|=4;       //blue
  if(er& UNSTKERR)r|=8;        //orange

  if(er& IMPRECISERR) {r|=16;pau=250;}
  if(er& PRECISERR)   {r|=32;pau=150;}
  if(er& IBUSERR)     {r|=64;pau=100;}
  
  // Go to infinite loop when Usage Fault exception occurs 
  while (1)
  {
    
    if(r&1) LSCAN2_LEDOn(LEDRED);
    if(r&2) LSCAN2_LEDOn(LEDGREEN);
    if(r&4) LSCAN2_LEDOn(LEDBLUE);
    if(r&8) LSCAN2_LEDOn(LEDORANGE);
    
    if(r&16) {LSCAN2_LEDOn(LEDRED);LSCAN2_LEDOn(LEDGREEN);}
    if(r&32) {LSCAN2_LEDOn(LEDGREEN);LSCAN2_LEDOn(LEDBLUE);}
    if(r&64) {LSCAN2_LEDOn(LEDBLUE);LSCAN2_LEDOn(LEDORANGE);}
    
    it_mDelay(pau);
    LSCAN2_LEDOff(LEDRED);
    LSCAN2_LEDOff(LEDGREEN);
    LSCAN2_LEDOff(LEDBLUE);
    LSCAN2_LEDOff(LEDORANGE);
    it_mDelay(pau);
  }
}
*/
/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
#define CFSR_UNDEFINSTR_BIT 16
#define CFSR_INVSTATE_BIT 17
#define CFSR_INVPC_BIT 18
#define CFSR_NOPC_BIT 19

#define CFSR_UNALIGNED_BIT 24
#define CFSR_DIVBZERO_BIT24

#define CFSR_UNDEFINSTR ((uint32_t)1<<16)
#define CFSR_INVSTATE ((uint32_t)1<<17)
#define CFSR_INVPC ((uint32_t)1<<18)
#define CFSR_NOPC ((uint32_t)1<<19)

#define CFSR_UNALIGNED ((uint32_t)1<<24)
#define CFSR_DIVBZERO ((uint32_t)1<<25)
/*
void UsageFault_Handler(void)
{
  uint32_t er=SCB->CFSR;
  uint32_t pau=500;
  
  uint8_t r=0;//0000[obgr] leds
  if(er& CFSR_UNDEFINSTR)r|=1;  //red
  if(er& CFSR_INVSTATE)r|=2;    //green
  if(er& CFSR_INVPC)r|=4;       //blue
  if(er& CFSR_NOPC)r|=8;        //orange

  if(er& CFSR_UNALIGNED){r|=3;pau=250;}
  if(er& CFSR_DIVBZERO){r|=12;pau=150;}

  
  // Go to infinite loop when Usage Fault exception occurs
  while (1)
  {
    
    if(r&1) LSCAN2_LEDOn(LEDRED);
    if(r&2) LSCAN2_LEDOn(LEDGREEN);
    if(r&4) LSCAN2_LEDOn(LEDBLUE);
    if(r&8) LSCAN2_LEDOn(LEDORANGE);
    it_mDelay(pau);
    LSCAN2_LEDOff(LEDRED);
    LSCAN2_LEDOff(LEDGREEN);
    LSCAN2_LEDOff(LEDBLUE);
    LSCAN2_LEDOff(LEDORANGE);
    it_mDelay(pau);
  }
}
*/
/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void) //in tsvc.s-assembler code
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*
void PendSV_Handler(void)
{
}
*/


/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
/*void EXTI0_IRQHandler(void)
{
  ;
}*/


/**
* @brief  mDelay
*         This function provides delay time in milli sec
* @param  msec : Value of delay required in milli sec
* @retval None
*/
void it_mDelay (uint32_t msec)
{
  //it_uDelay(msec * 1000);   
  uDelay(msec * 1000);  
}
/**
* @}
*/ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
