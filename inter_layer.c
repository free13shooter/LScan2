/**
  ******************************************************************************
  * @file    inter_layer.c
  * @author  MW dem1305
  * @version V1.0.0
  * @date    19.3.2014
  *                 интерфейс уровня приложений
  ******************************************************************************
  * IF_cb of applications layer
  ******************************************************************************
  */ 

#ifdef USB_OTG_HS_DMA_ENABLED 
#pragma     data_alignment = 4 
#endif /* USB_OTG_HS_IF_cbNAL_DMA_ENABLED */

/* Includes ------------------------------------------------------------------*/
#include "usbd_inter_layer.h"
#include "Hardware.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* These are external variables imported from Streaming core to be used for IN 
   transfer management. */
extern uint8_t  APP_Tx_Buffer []; /* Write Streaming received data in this buffer.
                                   //  These data will be sent over USB IN endpoint
                                   //  in the Streaming core functions. */
extern uint32_t APP_Tx_ptr_in;    /* Increment this pointer or roll it back to
                                   //  start address when writing received data
                                   //  in the buffer APP_Tx_Buffer. */
extern uint8_t USB_Rx_Buffer[];
extern uint8_t USB_IsoRx_Buffer[];
/* Private function prototypes -----------------------------------------------*/


//нужно для вызова из usbd_custom_core.c - интерфейс приложений
CUSTOM_IF_Prop_TypeDef IF_cbfs_fops = 
{
  IF_cb_Init,
  IF_cb_DeInit,
  IF_cb_Ctrl,
  IF_cb_IsoRx,
  IF_cb_DataTx,
  IF_cb_DataRx
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  IF_cb_Init
  *         Initializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t IF_cb_Init(void)
{ 
  return USBD_OK;
}

/**
  * @brief  IF_cb_DeInit
  *         DeInitializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t IF_cb_DeInit(void)
{
  return USBD_OK;
}


static uint16_t IF_cb_DataTx(uint8_t* Buf, uint32_t Len)
{
  return USBD_OK;
}
//---------- управление ---------------------------
/**
  * @brief  IF_cb__Ctrl
  *         Manage the vendor class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t IF_cb_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
  switch (Cmd)
  {
  case 0:
    /* Not  needed for this driver */
    break;

  
    
  default:
    break;
  }

  return USBD_OK;
}


//----------обратка для изохронного приема----------------
static uint16_t IF_cb_IsoRx (uint8_t* Buf, uint32_t Len)
{ 
  return USBD_OK;
}
//------------------------------------------------------------------------

/**
  * @brief  IF_cb_DataTx
  *         Streaming received data to be send over USB IN endpoint are managed in 
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else IF_cb_FAIL
  */
//интерфейс TX приложений по in interrupt
// смысла в копировании блоками нет,-на уровне ассемблера код разделения массива на 
//блоки преобразуется в схожий этому коду

uint16_t DataTx (uint8_t* Buf, uint32_t Len)
{
	uint32_t i;
	//loop through buffer
	for( i = 0; i < Len; i++ )
	{
		APP_Tx_Buffer[APP_Tx_ptr_in] = (uint8_t) Buf[i];// буфер APP_Tx_Buffer используется драйвером USB      
		//increase pointer value
		APP_Tx_ptr_in++;
                
		/* To avoid buffer overflow */
		if(APP_Tx_ptr_in == APP_TX_DATA_SIZE)
		{
			APP_Tx_ptr_in = 0;
		}
	}
        
	return USBD_OK;
}

//------------------------------------------------------------------------
/**
  * @brief  IF_cb_DataRx
  *         Data received over USB OUT endpoint are sent over Streaming interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on Streaming interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else IF_cb_FAIL
  */
uint16_t IF_cb_DataRx (uint8_t* Buf, uint32_t Len)
{
	return USBD_OK;
}
//------------------------------------------------------------------------


//вывод символов в порт
int putchar(int c)
{
  APP_Tx_Buffer[APP_Tx_ptr_in] = (uint8_t) c;
  //increase pointer value
  APP_Tx_ptr_in++;
  /* To avoid buffer overflow */
  if(APP_Tx_ptr_in == APP_TX_DATA_SIZE)
  {
    APP_Tx_ptr_in = 0;
  }
  return c;
}
//------------------------------------------------------------------------

//--------------Delays---------------------------------------------------------
/**
* @brief  uDelay
*         This function provides delay time in micro sec
* @param  usec : Value of delay required in micro sec
* @retval None
*/
void uDelay (uint32_t usec)
{
  uint32_t count = 0;
  const uint32_t utime = (120 * usec / 7);
  do
  {
    if ( ++count > utime )
    {
      return ;
    }
  }
  while (1);
}


/**
* @brief  mDelay
*         This function provides delay time in milli sec
* @param  msec : Value of delay required in milli sec
* @retval None
*/
void mDelay (uint32_t msec)
{
  uDelay(msec * 1000);   
}
/**
* @}
*/ 

/******************* MW dem1305 *************************END OF FILE****/
