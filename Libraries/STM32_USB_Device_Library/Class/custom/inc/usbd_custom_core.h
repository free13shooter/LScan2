/**
  ******************************************************************************
  * @file    usbd_hid_core.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   header file for the usbd_hid_core.c file.
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

#ifndef __USB_CUSTOM_CORE_H_
#define __USB_CUSTOM_CORE_H_

#include  "usbd_ioreq.h"
#include  "stm32f4xx_it.h"
#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */

#include <stdlib.h>
//#include  "LScan_protocol.h" //****************************************LScan******************************************
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
/*  
#define BOOL  uint8_t
#define FALSE 0
#define TRUE  1
*/
//----------------------

#define USB_CUSTOM_CONFIG_DESC_SIZ       39 //����� ����� � �����������
#define USB_CUSTOM_DESC_SIZ              9

#define CUSTOM_DESCRIPTOR_TYPE           0x21

#define CUSTOM_REQ_SET_PROTOCOL          0x0B
#define CUSTOM_REQ_GET_PROTOCOL          0x03

#define CUSTOM_REQ_SET_IDLE              0x0A
#define CUSTOM_REQ_GET_IDLE              0x02

//--------------BUFFERS--------------------------------------------------------
/* Total size of the out USB RX ISOCHRONOUS STREAM transfer buffer */
#define USB_ISO_OUT_PACKET_NUM      16 	//������� � ������ �� ISOC_OUT_PACKET_SIZE
#define USB_ISO_RX_DATA_SIZE  ((uint32_t)(ISOC_OUT_PACKET_SIZE * USB_ISO_OUT_PACKET_NUM))

/* Total size of the out USB RX transfer buffer */
#define USB_OUT_PACKET_NUM          8 
#define USB_RX_DATA_SIZE  ((uint32_t)(OUT_PACKET_SIZE * USB_OUT_PACKET_NUM))// ����

/* Total size of the Application TX transfer buffer */
#define IN_PACKET_NUM               8 
#define APP_TX_DATA_SIZE  ((uint32_t)(IN_PACKET_SIZE * IN_PACKET_NUM))//������ ������ TX

extern uint8_t USB_IsoRx_Buffer [USB_ISO_RX_DATA_SIZE] ;//���������������� ��� ��������� ������� ������
extern uint16_t LS_BUF_SIZE;//������ ����� ������,LScan(USB_ISO_RX_DATA_SIZE-�����)
extern uint8_t* LS_BUF_BEG;//ISO	������ ���� ������
extern uint8_t* LS_BUF_END;//��������� ���� ������
extern uint8_t* iso_buff_over;

extern volatile uint8_t* IsocOutWrPtr;//��������� ������
extern volatile uint8_t* IsocOutRdPtr;//��������� ������
extern volatile uint16_t ISO_Buff_Size;
extern volatile uint16_t RX_Buff_Size;
//-----------------------��������� �����--------------------------------------//
//-----------------����������� ����� 0x00/0x80--------------------------------//
            /* Total size of the Command (control) buffer */
#define CMD_PACKET_NUM             64 
#define CMD_BUFF_SIZE  ((uint32_t)(CMD_PACKET_SIZE * CMD_PACKET_NUM))//������ ������
//-----------------------------------------------------------------------------
//������� ���������� ���������� ����� EP0 OUT � ���� usb_setup_req

/**************************************************/
/*  control Requests                              */
/**************************************************/
#define NO_CMD                0x00  //��� �������
#define CMD_SAVE              0x01  //������ � ��������� �����
#define CMD_CORE              0x02  //��������� ������� ���� USB (������-�������)
#define CMD_RESERVED1         0x03  //���������������
#define CMD_RESERVED2         0x04  //���������������
#define CMD_STALL_EP	        0x05  //������������� �����//wIndex: 0-IN_EP,1-OUT_EP,2-ISOC (ep 0 ������ ����������)
#define CMD_RESUME_EP	        0x06  //����������� ������ �����//wIndex: 0-IN_EP,1-OUT_EP,2-ISOC
#define CMD_DIS_TX	          0x07  //������ TX ������� �� IN_EP
#define CMD_EN_TX             0x08  //���������� TX ������� �� IN_EP
#define CMD_DEVICE_RESET	0x09	//����� �����������
#define CMD_DEVICE_DISCONNECT	0x10	//���������� �� USB-�����

#define CMD_GET_EP_PK_SIZE    0x85  //�������� ������ ������ �����,������-�����:
                                    //wIndex:0-IN_EP,1-OUT_EP,2-ISOC,3-EP0, default-error
#define CMD_SET_EP_PK_SIZE    0x89  //���������� ������ ������ �����,������-�����:
                                    //wIndex:0-IN_EP,1-OUT_EP,2-ISOC,3-EP0, default-error,wValue - ������,������� 2 ����� ������ �������
#define CMD_RECEIVE           0x86  //�������� ������ �� ������ ������ 
#define CMD_GET_EP_BUF_SIZE   0x87  //�������� ������ ������ �����,������-�����(4 �����)
#define CMD_GET_EP_BUFF_PDIST 0x88  //�������� ��������� ����� ����������� ������/������ � ������



/**     
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */


/**
  * @}
  */ 

typedef struct _CUSTOM_IF_PROP
{
  uint16_t (*pIf_Init)     (void);   
  uint16_t (*pIf_DeInit)   (void);
  uint16_t (*pIf_Ctrl)     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);//����������
  uint16_t (*pIf_IsoRx)    (uint8_t* Buf, uint32_t Len);//����� ���������� 
  uint16_t (*pIf_DataTx)   (uint8_t* Buf, uint32_t Len);//�������� ���������
  uint16_t (*pIf_DataRx)   (uint8_t* Buf, uint32_t Len);//����� ��������� 
}
CUSTOM_IF_Prop_TypeDef;
/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
  
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_Class_cb_TypeDef  USBD_CUSTOM_cb;

extern USBD_Usr_cb_TypeDef USBD_User_cb;

/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 


/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
//������ �� ��������� _fr �� ��������� _to � ��������� ������ �������� _buf_size
#define _bytesFromTo(_fr,_to,_buf_size) ((_fr)<=(_to)?(int)((_to)-(_fr)):((int)(_buf_size)-(int)(_fr)+(int)(_to)))
//��������� ����� � ������
/*#define _freeIsoSteamBuffSize   (IsocOutWrPtr>=IsocOutRdPtr?(uint32_t)IsocOutWrPtr-(uint32_t)IsocOutRdPtr: \
        (uint32_t)IsocOutWrPtr-(uint32_t)USB_IsoRx_Buffer+(uint32_t)USB_IsoRx_Buffer+  \
          (uint32_t)LS_BUF_SIZE-(uint32_t)IsocOutRdPtr)
*/

void EP0_TX(USB_OTG_CORE_HANDLE* pdev,uint8_t* buff,uint32_t len);

typedef struct
{
  uint8_t* buff;
  uint32_t len;
}cmd_task;

typedef void cmd_ty_tx_cb(cmd_task* context);//callback for audio timer

void push_cmd_tx_task(uint8_t* buff,uint32_t len);

#endif  // __USB_CUSTOM_CORE_H_
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
