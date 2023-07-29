/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   USB Device configuration file
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
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

/* Includes ------------------------------------------------------------------*/
#include   "Hardware.h"

/** @defgroup USB_CONF_Exported_Defines
  * @{
  */ 


#define USBD_CFG_MAX_NUM           1 //configurations
#define USBD_ITF_MAX_NUM           1 //interfaces

#define USB_MAX_STR_DESC_SIZ       64 //length of strings descriptors



#define USBD_DYNAMIC_DESCRIPTOR_CHANGE_ENABLED 

/** @defgroup USB_String_Descriptors
  * @{
  */ 


/** @defgroup USB_CUSTOM_Class_Layer_Parameter
  * @{
  */ 
//----------------------------------------------------------------------------//
#define CMD_EP                    0x00  //интеррупт   командная
#define IN_EP                     0x81  //интеррупт   64 байт
#define OUT_EP                    0x02  //интеррупт   64 байт
#define ISOC_OUT_EP               0x03  //изохронная  512 байт
//----------------------------------------------------------------------------//
//1-нет синхронизации 5-асинхронная 9-адаптивная 13-синхронная                //

#define ISOC_OUT_TYPE             1   //тип(битовые поля)изохронной точки  
//----------------------------------------------------------------------------//
#define CMD_PACKET_SIZE           USB_OTG_MAX_EP0_SIZE   // TX0_FIFO_FS_SIZE

#define IN_PACKET_SIZE            64  //64/4=16 TX1_FIFO_FS_SIZE
#define OUT_PACKET_SIZE           64  //
#define ISOC_OUT_PACKET_SIZE      832 //кратно 64 !!!
//----------------------------------------------------------------------------//
#define IN_EP_FRAME_INTERVAL      0x01
#define OUT_EP_FRAME_INTERVAL     0x01
#define ISOC_EP_FRAME_INTERVAL    0x01
//----------------------------------------------------------------------------//

//подпрограммы обратного вызова взаимодействия с классом.
//блокировка вызвавших прерываний до выхода
#define APP_FOPS                        IF_cbfs_fops

/**
  * @}
  */ 
/** @defgroup USB_CONF_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_CONF_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 


#endif //__USBD_CONF__H__

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

