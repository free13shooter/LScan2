/**
********************************************************************************
* @file    usbd_custom_core.c
* @author  MW dem1305
* @version V1.0.0
* @date    
* @brief   This file provides the stream core functions.
*
* 
*full control isochronous stream.cmd functions.endpoints:3-isoOut;1,2-int In/Out
*control over EP0 
********************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"

//-------------------------------------------------------------------------//
//________________________LSCAN____________________________________________//
#include "LScan.h"



/*
arm_pid_instance_f32 XPID_param;
arm_pid_instance_f32 YPID_param;
uint8_t* ptrPIDparam;
arm_pid_instance_f32* ptrPIDparamSet;
float32_t* float32_tPtr;//указатель
*/
//-------------------------------------------------------------------------//
__IO uint32_t FullSysReset=0;
__IO uint32_t USB_Disconnect=0;
//смещение	указателя в кольцевом буфере
uint8_t* IncrementIntOutWriteBuffPointer();
//------------------------------------------------------------------------------
extern uint8_t EnableUSB;//разрешить USB
extern USB_OTG_CORE_HANDLE  USB_OTG_dev;//usb core device
/*********************************************
   Custom Device library callbacks
 *********************************************/
static uint8_t  USBD_CUSTOM_Init (void  *pdev,uint8_t cfgidx);
static uint8_t  USBD_CUSTOM_DeInit (void  *pdev,uint8_t cfgidx);
static uint8_t  USBD_CUSTOM_Setup (void  *pdev,USB_SETUP_REQ *req);
static uint8_t  USBD_CUSTOM_DataIn (void  *pdev, uint8_t epnum);
static uint8_t  USBD_CUSTOM_DataOut (void  *pdev, uint8_t epnum);
static uint8_t  USBD_CUSTOM_SOF (void  *pdev);
static uint8_t  IsoOUTIncomplete (void  *pdev);
/*********************************************
   
 *********************************************/
static uint8_t  *USBD_CUSTOM_GetCfgDesc (uint8_t speed, uint16_t *length);

//============================================
uint16_t melUsbInit[]={1500,100,2000,150};
//_________________________________________________________________________________________________
#pragma data_alignment=4   
uint8_t APP_Tx_Buffer [APP_TX_DATA_SIZE] ; 

uint32_t APP_Tx_ptr_in  = 0;//точка ввода
uint32_t APP_Tx_ptr_out = 0;//точка вывода
uint32_t APP_Tx_length = 0;//осталось передать
uint8_t  USB_Tx_State = 0;//передача в процессе?
uint8_t  doNotBreakTX=0;//не передавать нулевой пакет(завершение)

uint8_t DisableTX=1;//флаг запрещения передачи через EP INT IN(устанавливает драйвер)
//_________________________________________________________________________________________________
//                         INT_OUT
//                      БУФЕР ПРИЕМА
#pragma data_alignment=4   
uint8_t USB_Rx_Buffer [USB_RX_DATA_SIZE] ;

uint8_t* IntOutWrPtr = USB_Rx_Buffer;
uint8_t* IntOutRdPtr = USB_Rx_Buffer;
uint32_t APP_IsoRX_cnt;//счетчик байтов

extern IMODE ICmode;//тип соединения
//_________________________________________________________________________________________________
//                             ISO_OUT * * * * * * * * * * * * * * * * * * * * * * * * *
//                    БУФЕР ИЗОХРОННОГО ПРИЕМА
#pragma data_alignment=4   
uint8_t USB_IsoRx_Buffer [USB_ISO_RX_DATA_SIZE];//+ISOC_OUT_PACKET_SIZE] ;//переопределяется при изменении размера пакета

volatile uint8_t* IsocOutWrPtr  = USB_IsoRx_Buffer;
volatile uint8_t* IsocOutRdPtr  = USB_IsoRx_Buffer;
volatile uint16_t ISO_Buff_Size = (uint16_t)(USB_ISO_RX_DATA_SIZE);//основной размер (кратно 64)
volatile uint16_t RX_Buff_Size  = USB_RX_DATA_SIZE;

//размеры пакетов точек
uint16_t IntInPacketSize=IN_PACKET_SIZE;
uint16_t IntOutPacketSize=OUT_PACKET_SIZE;
uint16_t IsocOutPacketSize=(uint16_t)(ISOC_OUT_PACKET_SIZE);//размер пакета (кратно 64)
uint16_t CmdPacketSize=USB_OTG_MAX_EP0_SIZE;
//LScan
uint16_t LS_BUF_SIZE=(uint16_t)(USB_ISO_RX_DATA_SIZE);//основной размер: часть буфера,LScan(USB_ISO_RX_DATA_SIZE-пакет)
uint8_t* LS_BUF_BEG=USB_IsoRx_Buffer;//первый байт буфера
uint8_t* LS_BUF_END=USB_IsoRx_Buffer+USB_ISO_RX_DATA_SIZE-1;//последний байт буфера

uint8_t* iso_buff_over=USB_IsoRx_Buffer+(uint32_t)(USB_ISO_RX_DATA_SIZE);
//_________________________________________________________________________________________________
//                           CMD_IN/OUT * * * * * * * * * * * * * * * * * * * * * * * * *
//                ######### КОМАНДНЫЙ БУФЕР ###########
//используется управляющая командная точка 0x00/0x80
#pragma data_alignment=4   
uint8_t CmdBuff [CMD_BUFF_SIZE] ;//интервал 1мс
uint8_t* CmdWrPtr = CmdBuff;
uint8_t* CmdRdPtr = CmdBuff;

volatile uint8_t Cmd = NO_CMD;
uint32_t CmdLen = 0;
/*
uint8_t   bmRequest=0;//req->bmRequest;  //тип запроса(селектор в usbd_core.c)=VENDOR                    
uint8_t   bRequest=0;//req->bRequest;   //код запроса                          
uint16_t  wValue=0;//req->wValue;     //доп.аргумент                              
uint16_t  wIndex=0;//req->wIndex;     //индекс                             
uint16_t  wLength=0;//req->wLength;   //длина буфера принимаемых от/отдаваемых хосту данных
*/



extern uint8_t LScanStatus;
extern uint8_t R,G,B;	  //целевые цвета
//_________________________________________________________________________________________________
//============================================
static void Handle_USBAsynchXfer (void *pdev);// передача TX


__IO uint8_t PlayFlag=0;
//__IO uint32_t sampleBytes=0;//количество принятых байтов 

//________________________________________

/*********************************************
   
 *********************************************/

/**
  * @}
  */ 
extern CUSTOM_IF_Prop_TypeDef  APP_FOPS;


/** @defgroup USBD_HID_Private_Variables
  * @{
  */ 
/*********************************************
   
 *********************************************/
USBD_Class_cb_TypeDef  USBD_CUSTOM_cb = 
{
  USBD_CUSTOM_Init,
  USBD_CUSTOM_DeInit,
  USBD_CUSTOM_Setup,
  NULL,//USBD_CUSTOM_EP0_TxReady, /*EP0_TxSent*/  
  NULL,//USBD_CUSTOM_EP0_RxReady, /*EP0_RxReady*/
  USBD_CUSTOM_DataIn, /*DataIn передача*/
  USBD_CUSTOM_DataOut, /*DataOut приём*/
  USBD_CUSTOM_SOF,  /*SOF */
  NULL,             //IsoINIncomplete
  IsoOUTIncomplete, /*DataOut isoc приём*/   
  USBD_CUSTOM_GetCfgDesc,
#ifdef USB_OTG_HS_CORE  
  USBD_CUSTOM_GetCfgDesc, /* use same config as per FS */
#endif  
};

#pragma data_alignment=4   
/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_CfgDesc[USB_CUSTOM_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
  USB_CUSTOM_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing  the configuration*/
  0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  
  /************** Descriptor of interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x03,         /*bNumEndpoints*/
  0xFF,         /*bInterfaceClass*/
  0xFF,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0xFF,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0x00,         /*iInterface: Index of string descriptor*/
  
  /******************** Descriptor of endpoint IN ********************/
  /* 18 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
  
  IN_EP,     /*bEndpointAddress: Endpoint Address (IN)*/
  3,          /*bmAttributes: Interrupt endpoint*/
  IN_PACKET_SIZE, /*wMaxPacketSize: 64 Byte max */
  0x00,
  IN_EP_FRAME_INTERVAL,          /*bInterval: Polling Interval (10 ms)*/
  
  /******************** Descriptor of endpoint OUT ********************/
  /* 25 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
  
  OUT_EP,     /*bEndpointAddress: Endpoint Address (IN)*/
  3,          /*bmAttributes: Interrupt endpoint*/
  OUT_PACKET_SIZE, /*wMaxPacketSize: 64 Byte max */
  0x00,
  OUT_EP_FRAME_INTERVAL,          /*bInterval: Polling Interval (10 ms)*/
    
  /******************** Descriptor of endpoint ISOC OUT ********************/
  /* 32 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
  
  ISOC_OUT_EP,     /*bEndpointAddress: Endpoint Address (IN)*/
  ISOC_OUT_TYPE,          /*bmAttributes: isochronous endpoint*/
  LOBYTE(ISOC_OUT_PACKET_SIZE), /*wMaxPacketSize: 1023 Byte max */
  HIBYTE(ISOC_OUT_PACKET_SIZE),
  ISOC_EP_FRAME_INTERVAL,          /*bInterval: Polling Interval (1 ms)*/
  
  
  /* 39 total */
} ;

//============================================
static uint8_t  USBD_CUSTOM_Init (void  *pdev, 
                               uint8_t cfgidx)
{
  APP_FOPS.pIf_Init();//обнуление счетчиков пакетов
  DisableTX=1;//запретить TX
  IntInPacketSize=IN_PACKET_SIZE;
  IntOutPacketSize=OUT_PACKET_SIZE;
  IsocOutPacketSize=ISOC_OUT_PACKET_SIZE;
  //-------------------------------  
  /* Open EP IN */
  DCD_EP_Open(pdev,IN_EP,IntInPacketSize,USB_OTG_EP_INT); 
  //--------ISOC-------------------
  //LS_BUF_SIZE=ISO_Buff_Size-IsocOutPacketSize;
  //LS_BUF_BEG=USB_IsoRx_Buffer;
 // LS_BUF_END=USB_IsoRx_Buffer+LS_BUF_SIZE-1;
  //IsocOutRdPtr=IsocOutWrPtr=USB_IsoRx_Buffer;
  //sampleBytes=0;
  /*for (uint8_t* pt=USB_IsoRx_Buffer;pt<USB_IsoRx_Buffer+ISO_Buff_Size;pt++)
  {
    pt[0]=0;
  }
  PlayFlag=0;*/
  iso_buff_over=USB_IsoRx_Buffer+LS_BUF_SIZE;
  DCD_EP_Open(pdev,ISOC_OUT_EP,IsocOutPacketSize,USB_OTG_EP_ISOC);
  DCD_EP_PrepareRx(pdev,ISOC_OUT_EP,(uint8_t*)IsocOutWrPtr,IsocOutPacketSize);
  
  //-------------------------------
  /* Open EP OUT */
  DCD_EP_Open(pdev,OUT_EP,IntOutPacketSize,USB_OTG_EP_INT);
  DCD_EP_PrepareRx(pdev,OUT_EP,(uint8_t*)USB_Rx_Buffer,IntOutPacketSize);
  
  BeepMelody(melUsbInit,(uint16_t)2);
  ICmode=IM_USB;
  return USBD_OK;
}
//============================================
static uint8_t  USBD_CUSTOM_DeInit (void  *pdev,uint8_t cfgidx)
{
  
  DCD_EP_Close (pdev , IN_EP);
  DCD_EP_Close (pdev , OUT_EP);
  DCD_EP_Close (pdev , ISOC_OUT_EP);
  
  return USBD_OK;
}
//============================================
//см.usbd_core.c,usbd_ioreq.c
/*
typedef  struct  usb_setup_req {
    
    uint8_t   bmRequest;  //тип запроса(селектор в usbd_core.c)=VENDOR                    
    uint8_t   bRequest;   //код запроса                          
    uint16_t  wValue;     //доп.аргумент                              
    uint16_t  wIndex;     //индекс                             
    uint16_t  wLength;    //длина буфера принимаемых от/отдаваемых хосту данных                            
} USB_SETUP_REQ;
*/
static uint8_t  USBD_CUSTOM_Setup (void  *pdev,USB_SETUP_REQ *req)
{
  LSCAN2_LEDOn(LEDGREEN);
  
  USB_OTG_CORE_HANDLE *pde=pdev;
  
  Cmd=req->bRequest;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)//&0x60
  {
  //.......
  case USB_REQ_TYPE_VENDOR : 
      if (req->bmRequest & 0x80)//направление - от устройства к хосту(IN DIRECTION)
        {
          Cmd = req->bRequest;
         switch(Cmd)
         {
          //-----------получить расстояние между указателями в буфере точки 4 байта------------
          case CMD_GET_EP_BUFF_PDIST:
            switch(req->wIndex)//индекс точки ссылается на буфер
            {
            case 3:CmdLen =CMD_BUFF_SIZE;break;
            case 0:CmdLen =_bytesFromTo(APP_Tx_ptr_in,APP_Tx_ptr_out,APP_TX_DATA_SIZE);break;
            case 1:CmdLen =_bytesFromTo(IntOutWrPtr,IntOutRdPtr,USB_RX_DATA_SIZE);break;
            case 2:CmdLen =_bytesFromTo(IsocOutWrPtr,IsocOutRdPtr,ISO_Buff_Size);break;
            default:CmdLen =CMD_BUFF_SIZE;break;
            }
            USBD_CtlSendData (pdev,(uint8_t*)&CmdLen,4);
            Cmd=NO_CMD;
            return USBD_OK;
          //----------  
          case CMD_GET_EP_PK_SIZE: //передать размер пакета точки(2 байта)
            switch(req->wIndex)
            {
            case 3:CmdLen =CmdPacketSize;break;
            case 0:CmdLen =IntInPacketSize;break;
            case 1:CmdLen =IntOutPacketSize;break;
            case 2:CmdLen =IsocOutPacketSize;break;
            default:CmdLen =0;break;//error
            }
            CmdBuff[0]=LOBYTE(CmdLen);CmdBuff[1]=HIBYTE(CmdLen);
            USBD_CtlSendData (pdev,CmdBuff,2);
            Cmd=NO_CMD;
            return USBD_OK;
					//----------  
          case CMD_SET_EP_PK_SIZE: //установить и передать размер пакета точки(2 байта)
            switch(req->wIndex)
            {
            case 0:if(req->wValue>IN_PACKET_SIZE)req->wValue=IN_PACKET_SIZE;CmdLen =IntInPacketSize=req->wValue;break;
            case 1:if(req->wValue>OUT_PACKET_SIZE)req->wValue=OUT_PACKET_SIZE;CmdLen =IntOutPacketSize=req->wValue;break;
            case 2:if(req->wValue>ISOC_OUT_PACKET_SIZE)req->wValue=ISOC_OUT_PACKET_SIZE;
						PlayFlag=0;
						CmdLen =IsocOutPacketSize=req->wValue;
						ISO_Buff_Size=USB_ISO_OUT_PACKET_NUM*IsocOutPacketSize;//переинициализация буфера
						IsocOutRdPtr=IsocOutWrPtr = USB_IsoRx_Buffer;
            LS_BUF_SIZE=ISO_Buff_Size-IsocOutPacketSize;
            LS_BUF_BEG=USB_IsoRx_Buffer;
            LS_BUF_END=USB_IsoRx_Buffer+LS_BUF_SIZE-1;
	    DCD_EP_PrepareRx(pdev,
                     ISOC_OUT_EP,
                     (uint8_t*)(IsocOutWrPtr),
                     IsocOutPacketSize);
						break;
            default:CmdLen =0;break;//error
            }
            CmdBuff[0]=LOBYTE(CmdLen);CmdBuff[1]=HIBYTE(CmdLen);
            USBD_CtlSendData (pdev,CmdBuff,2);
            Cmd=NO_CMD;
            return USBD_OK;	
          //-----------------получить буфера точки----------------
          case CMD_GET_EP_BUF_SIZE: //передать размер буфера точки(2 байта)
            switch(req->wIndex)
            {
            case 3:CmdLen =CMD_BUFF_SIZE;break;
            case 0:CmdLen =APP_TX_DATA_SIZE;break;
            case 1:CmdLen =USB_RX_DATA_SIZE;break;
            case 2:CmdLen =ISO_Buff_Size;break;
            default:CmdLen =CMD_BUFF_SIZE;break;
            }
            USBD_CtlSendData (pdev,(uint8_t*)&CmdLen,4);
            Cmd=NO_CMD;
            return USBD_OK;  
          //---------------------------------------
          case CMD_RECEIVE: //передать данные из буфера команд
            USBD_CtlSendData(pdev,CmdBuff,req->wLength);
            Cmd=NO_CMD;
            return USBD_OK;
          //_________________________________LSCAN____IN<<< к хосту_________________________________
					default:
						return (LScanCmd (pdev,req));//обработчик команд OUT EP0 LScan *************************************************
          //=================== 
					/*
          default://неопределенная команда,нельзя обработать
            Cmd=NO_CMD;
            return USBD_FAIL;  
					*/
         } //switch cmd
        }//if
      //=============================================================
        else // //направление - от хоста к устройству (OUT DIRECTION)
        {
          // Set the value of the current command to be processed 
          Cmd = req->bRequest;
          CmdLen = req->wLength;
          uint8_t ep;
                    
          switch(Cmd)
          {
          case CMD_SAVE: //записать данные в буфер команд
            // Подготовить прием буфера по EP0
            USBD_CtlPrepareRx (pdev,CmdBuff,req->wLength);
            Cmd=NO_CMD;
            return USBD_OK;
					  
          case CMD_DIS_TX: //запрет TX передач по IN_EP
            if((pde->dev.device_status)!=USB_OTG_CONFIGURED)return USBD_OK;
            DisableTX=1;
            Cmd=NO_CMD;
            //DCD_EP_Flush(pdev,IN_EP);
            DCD_EP_Close(pdev , IN_EP);
            //DCD_EP_Tx (pdev,IN_EP,NULL,0);
            return USBD_OK;
            
          case CMD_EN_TX:
            DisableTX=0;
            Cmd=NO_CMD;//разрешение TX передач по IN_EP
            DCD_EP_Open(pdev,IN_EP,IntInPacketSize,USB_OTG_EP_INT); 
            return USBD_OK; 
          //-----------------приостановить/возобновить точку----------------
          case CMD_STALL_EP:
          case CMD_RESUME_EP: 
            switch(req->wIndex)
            {
            case 0:ep=IN_EP;break;
            case 1:ep=OUT_EP;break;
            case 2:ep=ISOC_OUT_EP;break;
            default:Cmd=NO_CMD;return USBD_FAIL;
            }
            if(Cmd==CMD_STALL_EP)DCD_EP_Stall(pdev,ep);else DCD_EP_ClrStall(pdev,ep); 
            Cmd=NO_CMD;
            return USBD_OK;
	case CMD_DEVICE_RESET:
	  FullSysReset=1;
	  return USBD_OK;
        case CMD_DEVICE_DISCONNECT:
	  USB_Disconnect=1;
	  return USBD_OK;
							
          //---------------------------------------
          //_________________________________LSCAN__OUT>>__к устройству_____________________________
						
          default:
	    return (LScanCmd (pdev,req));//обработчик команд OUT EP0 LScan *************************************************
	//=================== 	
					/*
          default://неопределенная команда,нельзя обработать
            Cmd=NO_CMD;
            return USBD_FAIL;*/
          } //switch cmd
                    
        }//else
     
  //.......   
  default://неопределенный тип запроса req->bmRequest ,нельзя обработать
     Cmd=NO_CMD;
     return USBD_FAIL;
  }//switch (req->bmRequest & USB_REQ_TYPE_MASK)
  //--------------------------------------------------------
}
//============================================
/**
  * @brief USBD_CUSTOM_EP0_RxReady
  * Данные получены на конечной точке управления EP0_OUT (RX)
  * @param pdev: экземпляр устройства
  * @retval состояние
  */
/*    
static uint8_t  USBD_CUSTOM_EP0_RxReady (void *pdev )
{
  LSCAN2_LEDOn(LEDGREEN);//GREEN
  
  if (Cmd != NO_CMD)
  {
    switch(Cmd)
    {
      //---------------LScan----------------------
			default:
	LScan_Ctrl(Cmd, CmdBuff, CmdLen);//передать в LScan(там обработчик)
			
      //------неизвестная команда--------------
			
			break;
    }
    
    // Reset the command variable to default value 
    Cmd = NO_CMD;
  }
   
  return USBD_OK;
}
*/
//============================================
/**
  * @brief USBD_CUSTOM_EP0_RxReady
  * Данные переданы на конечной точке управления EP0_OUT (TX)
  * @param pdev: экземпляр устройства
  * @retval состояние
  */
/*
static uint8_t USBD_CUSTOM_EP0_TxReady(void *pdev )
{
  return USBD_OK;
  if(EP0TX_ready(pdev))RED_LED_ON();else {GREEN_LED_ON();return USBD_BUSY;}
  if(CmdRdPtr!=CmdWrPtr)
  {
    uint32_t len=CmdRdPtr>CmdWrPtr?CmdBuff+CMD_BUFF_SIZE-CmdRdPtr:CmdWrPtr-CmdRdPtr;
    USBD_CtlSendData(pdev,CmdRdPtr,len);
    CmdRdPtr+=len;
    if(CmdRdPtr>=CmdBuff+CMD_BUFF_SIZE)CmdRdPtr=CmdBuff;
    if(CmdRdPtr!=CmdWrPtr)return USBD_BUSY;
  }
  return USBD_OK;
}
*/
//============================================
static uint8_t  *USBD_CUSTOM_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (USBD_CUSTOM_CfgDesc);
  return USBD_CUSTOM_CfgDesc;
}
//============================================
/**
  * @brief  USBD_CUSTOM_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_CUSTOM_DataIn (void  *pdev,uint8_t epnum)  //передача данных
{
  LSCAN2_LEDOn(LEDRED);//RED
  
  
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;

  if (USB_Tx_State == 1)//в процессе
  {
    if (APP_Tx_length == 0||(DisableTX==1))  //все передано или запрещено
    {
      USB_Tx_State = 0; //свободен
      doNotBreakTX=0;//можно отдавать пустые пакеты
    }
    else 
    {
      if (APP_Tx_length > IntInPacketSize){
        USB_Tx_ptr = APP_Tx_ptr_out;
        USB_Tx_length = IntInPacketSize;
        
        APP_Tx_ptr_out += IntInPacketSize;
        APP_Tx_length -= IntInPacketSize;    
      }
      else //остался последний кусок
      {
        USB_Tx_ptr = APP_Tx_ptr_out;
        USB_Tx_length = APP_Tx_length;
        
        APP_Tx_ptr_out += APP_Tx_length;
        APP_Tx_length = 0;
      }
      
      /* готовим передачу через IN endpoint */
      DCD_EP_Tx (pdev,
                 IN_EP,
                 (uint8_t*)&APP_Tx_Buffer[USB_Tx_ptr],
                 USB_Tx_length);
    }
  }  
  
  return USBD_OK;
  
}
//=============================================DATA_OUT=============================================


static uint8_t  USBD_CUSTOM_DataOut (void  *pdev,uint8_t epnum)//прием данных
{
  uint32_t USB_Rx_Cnt;/* столько получили */
  int dist=0;
	
  USB_Rx_Cnt = ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;
	
  //==========================
  if(epnum==ISOC_OUT_EP)//ISOC
  {
    //GREEN_LED_ON();
    if(USB_Rx_Cnt&1){goto prepare_next;}//подготовить прием
    
    //критическая ошибка.пересечение указателя чтения (слишком медленное чтение)
    dist=_bytesFromTo(IsocOutWrPtr,IsocOutRdPtr,LS_BUF_SIZE);
    if(LScanStatus==STATUS_PLAY && dist<USB_Rx_Cnt)
    {
      IsocOutRdPtr=IsocOutWrPtr=USB_IsoRx_Buffer;
      //Beep(2000,90);//DBG
    }
    else
    {
      if(IsocOutWrPtr+USB_Rx_Cnt>=iso_buff_over)//вышел за предел,смещение на начало буфера
      {
        IsocOutWrPtr=IsocOutWrPtr+USB_Rx_Cnt-LS_BUF_SIZE;
        if(IsocOutWrPtr>USB_IsoRx_Buffer)
        {
          //((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame = 
          //(((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame)? 0:1;
   
          /* Prepare Out endpoint to receive next audio packet */
          //DCD_EP_PrepareRx(pdev,ISOC_OUT_EP,(uint8_t*)IsocOutWrPtr,IsocOutPacketSize);
          
          //DMA2_memcpy(DMA2_Stream2,USB_IsoRx_Buffer,iso_buff_over,IsocOutWrPtr-USB_IsoRx_Buffer);
          memcpy(USB_IsoRx_Buffer,iso_buff_over,IsocOutWrPtr-USB_IsoRx_Buffer);
          //return USBD_OK;
        }
      }
      else IsocOutWrPtr+=USB_Rx_Cnt;
    }
       	  
prepare_next:   	 
    /* Toggle the frame index */  
    ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame = 
      (((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame)? 0:1;
    
  /* Prepare Out endpoint to receive next audio packet */
  DCD_EP_PrepareRx(pdev,ISOC_OUT_EP,(uint8_t*)IsocOutWrPtr,IsocOutPacketSize);
    
  return USBD_OK;
  }//---------------------------------------------
  if(epnum==OUT_EP)
  { 
    LSCAN2_LEDOn(LEDGREEN);//GREEN
    /* Increment the buffer pointer */
    IntOutWrPtr += IntOutPacketSize;
			
    if (IntOutWrPtr >= (USB_Rx_Buffer + RX_Buff_Size))
    {/* All buffers are full: roll back */
      IntOutWrPtr = USB_Rx_Buffer;
    }
      
		/* USB data will be immediately processed, this allow next USB traffic being 
     NAKed till the end of the application Xfer */
    //APP_FOPS.pIf_DataRx(USB_Rx_Buffer, USB_Rx_Cnt);
       
    /* Prepare Out endpoint to receive next packet */
    DCD_EP_PrepareRx(pdev,
                     OUT_EP,
                     (uint8_t*)(IntOutWrPtr),
                     IntOutPacketSize);
    
  }
  
  return USBD_OK;
}  
//======================================ISO_OUT_INCOMPLETE==========================================
static uint8_t  IsoOUTIncomplete (void  *pdev)
{
  RED_LED_ON();
  return USBD_OK;  
}
//========
uint8_t IsEvenFrame (USB_OTG_CORE_HANDLE *pdev) 
{
  return !(USB_OTG_READ_REG32(&pdev->regs.HREGS->HFNUM) & 0x1);
}

uint32_t NumFrame (USB_OTG_CORE_HANDLE *pdev) //номер фрейма
{
  return (USB_OTG_READ_REG32(&pdev->regs.HREGS->HFNUM));
}
//==============================================SOF=================================================
static uint8_t  USBD_CUSTOM_SOF (void  *pdev)//старт фрейма
{
  //WHITE_LED_ON();
  
 /* if (PlayFlag==1)
  {      
    LScanStatus=STATUS_PLAY;
    PlaySample();
    //LScanStreamCmd((uint8_t*)(IsocOutRdPtr),LSCAN_CMD_PLAY);     // Command to be processed 
  }*/
   
  //----------------------------------------------------------------------------
  static uint16_t FrameCount = 0;
  //------------
  if (FrameCount++ >= IN_EP_FRAME_INTERVAL)
  {
    /* Reset the frame counter */
    FrameCount = 0;
    /* проверка наличия данных передачи для  IN pipe */
    if(DisableTX==0) Handle_USBAsynchXfer(pdev);
  }
	
  return USBD_OK;
}
//=================================================================================================
/**
  * @brief  Handle_USBAsynchXfer
  *         Send data to USB
  * @param  pdev: instance
  * @retval None
  */
static void Handle_USBAsynchXfer (void *pdev)
{
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
    
  uint32_t epstatus=DCD_GetEPStatus(pdev,IN_EP);//if USB_OTG_EP_TX_STALL
  //надо отдавать пустые пакеты,чтобы основная программа не заблокировалась ожидающими прерываниями
  if(DisableTX==1)
  {
    return;
  }
	//предотвращение передачи до конфигурации-
  //точку нельзя будет открыть,драйвер не стартанет:
      USB_OTG_CORE_HANDLE *pde=pdev;
      if((pde->dev.device_status)!=USB_OTG_CONFIGURED)return ;
  //--------------------------------------------------  
  if(USB_Tx_State != 1&&epstatus!=USB_OTG_EP_TX_STALL)
  {
    if (APP_Tx_ptr_out >= APP_TX_DATA_SIZE)// вышел за буфер
    {
      APP_Tx_ptr_out = 0;// в начало
    }
    //----------
    if(APP_Tx_ptr_out == APP_Tx_ptr_in) //достигли точки ввода или ничего не поступило или запрещено
    {
      USB_Tx_State = 0; //свободен
      if(doNotBreakTX==1) return;//еще нельзя передавать пустые пакеты(прервет передачу)
      //doNotBreakTX==0 - передача завершена
      DCD_EP_Tx (pdev,IN_EP,NULL,0);
      return;
    }
    //----------
    //дистанция в кольцевом буфере между указателями чтения и записи в байтах
    APP_Tx_length = _bytesFromTo(APP_Tx_ptr_out,APP_Tx_ptr_in,APP_TX_DATA_SIZE);
    if(APP_Tx_length>APP_TX_DATA_SIZE)APP_Tx_length=APP_TX_DATA_SIZE;
    //APP_Tx_length = APP_TX_DATA_SIZE - APP_Tx_ptr_out;
       
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
     APP_Rx_length &= ~0x03;
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
    //--------------------------------
    
    
    if (APP_Tx_length > IntInPacketSize)
    {
      USB_Tx_ptr = APP_Tx_ptr_out;
      USB_Tx_length = IntInPacketSize;
      
      APP_Tx_ptr_out += IntInPacketSize;	
      APP_Tx_length -= IntInPacketSize;
    }
    else
    {
      USB_Tx_ptr = APP_Tx_ptr_out;
      USB_Tx_length = APP_Tx_length;
      
      APP_Tx_ptr_out += APP_Tx_length;
      APP_Tx_length = 0;
    }
    USB_Tx_State = 1; //в работе
    
    DCD_EP_Tx (pdev,                   //передаем на старт
               IN_EP,
               (uint8_t*)&APP_Tx_Buffer[USB_Tx_ptr],
               USB_Tx_length);
    
    doNotBreakTX=1;//это не даст раньше времени передать нулевой пакет
    
    USB_Tx_State =0;//драйвер иногда зависает в ожидании данных,
    //если не обнулять статус процесса вывода 
    //и не заставить таким образом точку IN INTERRUPT
    //проанализировать снова указатели и произвести вывод
    //просто иногда драйвер слишком рано делает опрос,а данных
    //на вывод еще не существует.не рекомендуется отправлять свыше пакета
    //за один раз-повторный вывод перекроет данные,будет потеря.
    //------------------MW dem1305 06.4.2014-----------------------------
    
  }
 
}//static void Handle_USBAsynchXfer (void *pdev)

//========================================================================================== 

//========================================================================================== 
//смещение	указателя в кольцевом буфере 
uint8_t* IncrementIntOutWriteBuffPointer()
{
  // Increment the buffer pointer 
  IntOutWrPtr+=IntOutPacketSize;
  //проверка чтобы за границу не записало,иначе hard fault handler exception
  if (IntOutWrPtr > (USB_Rx_Buffer + USB_RX_DATA_SIZE))
  {
    IntOutWrPtr-= USB_RX_DATA_SIZE ;//сдвиг на излишек относительно начала
  }
 return IntOutWrPtr;
}

//------------------------------------------------------------------------------
/* 
static uint8_t EP0_ready(USB_OTG_CORE_HANDLE* pdev,uint8_t is_in)
{
  USB_OTG_EP *ep;
  ep = &pdev->dev.in_ep[0];
  
  ep->is_in = is_in?1:0;
  ep->num = 0;  
  
  uint32_t stat=USB_OTG_GetEPStatus(pdev ,ep);
  if(stat==((uint32_t)USB_OTG_EP_TX_VALID))return 1;
  return 0;
}
 
static uint8_t EP0RX_ready(USB_OTG_CORE_HANDLE* pdev)
{
  return EP0_ready(pdev,0);
}

static uint8_t EP0TX_ready(USB_OTG_CORE_HANDLE* pdev)
{
  return EP0_ready(pdev,1);
}  
//------------------------------------------------------------------------------
void EP0_TX(USB_OTG_CORE_HANDLE* pdev,uint8_t* buff,uint32_t len)
{
  if(len==0)return;
  push_cmd_tx_task(buff,len);
  return;
  push_in_cmd(buff,len);
  uint32_t bytes=_bytesFromTo(CmdRdPtr,CmdWrPtr,CMD_BUFF_SIZE);
  if(bytes==0)bytes=CMD_BUFF_SIZE;
  if(EP0TX_ready(pdev))USBD_CtlSendData(pdev,CmdRdPtr,bytes);
}

static void push_in_cmd(uint8_t* buff,uint32_t len)
{
  uint32_t left=len;
  uint8_t* p=buff;
  while(left)
  {
    *CmdWrPtr=*p;
    p++;CmdWrPtr++;left--;
    if(CmdWrPtr>=CmdBuff+CMD_BUFF_SIZE)CmdWrPtr=CmdBuff;
  }
}*/
//------------------------------------------------------------------------------
/*void push_cmd_tx_task(uint8_t* buff,uint32_t len)
{
  cmd_task* context=malloc(sizeof(cmd_task));
  uint8_t* buf=malloc(len);
  context->buff=buf;context->len=len;
  AddSystemTaskOneStart(cmd_deff_task_proc,context);
}


static void* cmd_deff_task_proc(void* ctx)
{
  if(ctx==NULL)return 0;
  uint8_t* buf=((cmd_task*)ctx)->buff;
  uint32_t len=((cmd_task*)ctx)->len;
  USBD_CtlSendData (&USB_OTG_dev,buf,len);
  if(buf && len)free(buf);
  free(ctx);
  return 0;
}
*/
//------------------------------------------------------------------------------
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
