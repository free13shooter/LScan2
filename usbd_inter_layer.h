/**
  ******************************************************************************
  * @file    usbd_inter_layer.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   Header for inter_layer.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_IF_cb_LAYER_H
#define __USBD_IF_cb_LAYER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include "usbd_custom_core.h"
#include "usbd_conf.h"


/* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the 
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */


/* Exported constants --------------------------------------------------------*/

#define DEFAULT_CONFIG                  0
#define OTHER_CONFIG                    1

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

static uint16_t IF_cb_Init     (void);
static uint16_t IF_cb_DeInit   (void);
static uint16_t IF_cb_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);//управление
static uint16_t IF_cb_IsoRx    (uint8_t* Buf, uint32_t Len);      
static uint16_t IF_cb_DataRx   (uint8_t* Buf, uint32_t Len);
static uint16_t IF_cb_DataTx   (uint8_t* Buf, uint32_t Len);


uint32_t GetRxCount();  //получить количество записанных байт c обнулением счетчика
uint32_t GetIsoRxCount();//для изохронной
int putchar(int c);     //вывод символов в TX
uint16_t DataTx   (uint8_t* Buf, uint32_t Len);////вывод в TX

//получить RX возврат количества переданных байт,
//вход-указатель приемного буфера и длина.
//счетчик обнуляется
uint32_t GetRx    (uint8_t* Buf, uint32_t Len);////получить RX
uint32_t GetIsoRx (uint8_t* Buf, uint32_t Len);//для изохронной

void uDelay (uint32_t usec);//microsec
void mDelay (uint32_t usec);//millisec

#endif /* __USBD_IF_cb_LAYER_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
