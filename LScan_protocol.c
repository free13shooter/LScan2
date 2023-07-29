/**
  Laser project by dem1305
  протокол обмена LScan,USB
  @2014
*/


/* Includes ------------------------------------------------------------------*/
#include "LScan_protocol.h"    //протокол обмена LScan ,usb верхний уровень

/* ---------------------------------------------------------------------------*/


/* Defines ------------------------------------------------------------------*/

/* Exports ------------------------------------------------------------------*/
extern CUSTOM_IF_Prop_TypeDef IF_cbfs_fops;

extern uint8_t USB_IsoRx_Buffer [] ;//БУФЕР ИЗОХРОННОГО ПРИЕМА
extern uint16_t USB_IsoLScanDSSsize_Buffer [];//вспомогательный буфер LScan для хранения размера пакета DSS(дискретизация)

extern uint8_t CmdBuff [] ;					//КОМАНДНЫЙ БУФЕР,интервал 1мс,используется управляющая командная точка 0x00/0x80
extern uint8_t USB_Rx_Buffer [] ;		//БУФЕР ИНТЕРРУПТ ПРИЕМА
extern uint8_t* IntOutWrPtr;
extern uint8_t* IntOutRdPtr;

extern uint8_t APP_Tx_Buffer [] ;		//БУФЕР ИНТЕРРУПТ ПЕРЕДАЧ 
extern uint32_t APP_Tx_ptr_in ;			//точка ввода
extern uint32_t APP_Tx_ptr_out ;		//точка вывода
extern uint8_t DisableTX;						//флаг запрещения передачи через EP INT IN(устанавливает драйвер)
extern uint16_t IsocOutPacketSize;  //

extern uint32_t Cmd;   //core
extern uint32_t CmdLen;//core

//формат воспроизведения из буфера USB_IsoRx_Buffer: размерность данных: 9 байт,младший опережает
extern RCC_ClocksTypeDef RCC_Clocks;
//extern int PlayDiscrFreq;//частота точек

uint8_t* ptrByte; //вспомогательный указатель
uint16_t* ptrWord;//вспомогательный указатель
uint32_t* ptrDWord;//вспомогательный указатель

extern uint8_t LScan_CMD;

uint8_t LScanStatus=STATUS_STOP;

extern uint8_t* BEG;	//первый байт буфера
extern uint8_t* END;	//последний байт буфера

extern uint16_t SampleDiscrInMs;//текущая дискретизация для 1 мс-семпла
extern __IO uint8_t PlayFlag;


extern int RGB_on_shift,RGB_off_shift;
extern float RGB_on_k,RGB_off_k;
extern float Linearity;
extern int R_power,G_power,B_power;
extern int R_pwm_min,G_pwm_min,B_pwm_min,R_pwm_max,G_pwm_max,B_pwm_max;
extern int Colors_PWM_Freq,R_Tresh,G_Tresh,B_Tresh;
extern uint8_t R,G,B;

extern uint16_t R_PWM[256];
extern uint16_t G_PWM[256];
extern uint16_t B_PWM[256];

extern uint16_t G_pwm_correct;

extern uint8_t TTL_modulation;
/* ---------------------------------------------------------------------------*/
uint16_t wIndex=0;
  
//---------- управление ---------------------------
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
uint16_t LScanCmd (void  *pdev,USB_SETUP_REQ *req)//обработчик команд OUT EP0 LScan
{
  Cmd=req->bRequest;
  uint16_t buf[2];
  uint32_t val32=0;
  uint16_t sampleBytes=(uint16_t)(_bytesFromTo(IsocOutRdPtr,IsocOutWrPtr,LS_BUF_SIZE));
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)//&0x60
  {
  //.......
  case USB_REQ_TYPE_VENDOR : 
      if (req->bmRequest & 0x80)//направление - от устройства к хосту(IN DIRECTION)
        {
          Cmd = req->bRequest;
          //---------------------------------------
          //задать параболический коэффициент
          if(Cmd>=(uint8_t)((uint8_t)SET_GET_SHIFT_K|(uint8_t)encOnR)&&
             Cmd<=(uint8_t)((uint8_t)SET_GET_SHIFT_K|(uint8_t)encOffB))
          {
            val32=((uint32_t)req->wValue|((uint32_t)req->wIndex<<16));
            
            switch(Cmd&=(uint8_t)~SET_GET_SHIFT_K)
            {
              case encOnRGBshift_k:memcpy(&RGB_on_k,&val32,4);USBD_CtlSendData (pdev,(uint8_t*)(&RGB_on_k),4);break;
              case encOffRGBshift_k:memcpy(&RGB_off_k,&val32,4);USBD_CtlSendData (pdev,(uint8_t*)(&RGB_off_k),4);break;
              default:Cmd=NO_CMD;return USBD_FAIL;//неправильные данные
            }
            Calculate_Colors_Shifting_and_Sampling();
            Cmd=NO_CMD;
            return USBD_OK;
          }
         //---------------------------------------
         switch(Cmd)
         {
          //_________________________________LSCAN__________________________________________________
	  case SET_GET_LINEARITY: //задать степень крутизны сдвига[1.01;100] (1.01-линейная функция) и получить назад значение для проверки
            val32=((uint32_t)req->wValue|((uint32_t)req->wIndex<<16));
            memcpy((void*)(&Linearity),&val32,4);USBD_CtlSendData (pdev,(uint8_t*)(&Linearity),4);
            Calculate_Colors_Shifting_and_Sampling();
            return USBD_OK; 
          //---------------------------------------
          case GET_LS_DISCR: //передать частоту дискретизации координат,2 байта discr и 2 байта количества данных в буфере
            buf[0]=(uint16_t)SampleDiscrInMs;//SampleDiscrInMs;//текущая дискретизация для 1 мс-семпла
            buf[1]=sampleBytes;
            //EP0_TX(pdev,(uint8_t*)buf,4);//4 байта(int)
            USBD_CtlSendData(pdev,(uint8_t*)buf,4);//4 байта(int)
            Cmd=NO_CMD;
            return USBD_OK; 
          //---------------------------------------
          case GET_STR_BUF_SIZE:   //передать размер потокового буфера  в байтах
            buf[0]=(uint16_t)(ISOC_OUT_PACKET_SIZE+ISO_Buff_Size);
            USBD_CtlSendData (pdev,(uint8_t*)&buf[0],2);//2 байта(uint16_t)
            Cmd=NO_CMD;
            return USBD_OK; 
          //---------------------------------------
          case SET_GET_TTL_MOD://задать режим модуляции TTL/аналоговый и получить назад значение для проверки 
            TTL_modulation=(uint8_t)(req->wValue);
            USBD_CtlSendData (pdev,(uint8_t*)(&TTL_modulation),1);
            Cmd=NO_CMD;
            return USBD_OK;
          //---------------------------------------
	  case SET_GET_COLOR_SHIFT://задать коэффициент периода сдвига и получить назад значение для проверки 
            switch(req->wIndex)
            {
              case encOnRGBshift:RGB_on_shift=(int)(req->wValue)*1000;USBD_CtlSendData (pdev,(uint8_t*)(&RGB_on_shift),4);break;
              case encOffRGBshift:RGB_off_shift=(int)(req->wValue)*1000;USBD_CtlSendData (pdev,(uint8_t*)(&RGB_off_shift),4);break;
              default:Cmd=NO_CMD;return USBD_FAIL;//неправильные данные
            }
            Calculate_Colors_Shifting_and_Sampling();
            Cmd=NO_CMD;
            return USBD_OK;
          //---------------------------------------
          case SET_GET_PWM_LIMIT://задать границу мощности и получить назад значение для проверки 
            switch(req->wIndex)
            {
              case encRmin:R_pwm_min=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&R_pwm_min),4);
              ReloadPWMColors();break;
              case encRmax:R_pwm_max=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&R_pwm_max),4);
              ReloadPWMColors();break;
              case encGmin:G_pwm_min=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&G_pwm_min),4);
              ReloadPWMColors();break;
              case encGmax:G_pwm_max=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&G_pwm_max),4);
              ReloadPWMColors();break;
              case encBmin:B_pwm_min=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&B_pwm_min),4);
              ReloadPWMColors();break;
              case encBmax:B_pwm_max=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&B_pwm_max),4);
              ReloadPWMColors();break;
              default:Cmd=NO_CMD;return USBD_FAIL;//неправильные данные
            }
            Cmd=NO_CMD;
            return USBD_OK;
          //---------------------------------------
          case SET_GET_LASER_POWER://задать яркость лазера и получить назад значение для проверки 
            switch(req->wIndex)
            {
              case RED_LASER:R_power=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&R_power),4);
              ReloadPWMColors();break;
              case GREEN_LASER:G_power=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&G_power),4);
              ReloadPWMColors();break;
              case BLUE_LASER:B_power=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&B_power),4);
              ReloadPWMColors();break;
              default:Cmd=NO_CMD;return USBD_FAIL;//неправильные данные
            }
            Cmd=NO_CMD;
            return USBD_OK; 
	  //---------------------------------------
	  case SET_GET_COLOR_FREQ://задать и получить значение частоты модуляции яркости
          {
            //int
            Colors_PWM_Freq=(int)(short)req->wValue;if(Colors_PWM_Freq<1)Colors_PWM_Freq=1;else if(Colors_PWM_Freq>168)Colors_PWM_Freq=168;
            USBD_CtlSendData (pdev,(uint8_t*)(&Colors_PWM_Freq),4);
            ReloadPWMColors();
            return USBD_OK;
          }
         //---------------------------------------
          case SET_GET_PWM_TRESH://задать порог мощности и получить назад значение для проверки
          {
            //int
            switch(req->wIndex)
            {
            case RED_LASER:R_Tresh=(int)(short)req->wValue;if(R_Tresh<0)R_Tresh=0;else if(R_Tresh>100)R_Tresh=100;
              ReloadPWMColors();USBD_CtlSendData (pdev,(uint8_t*)(&R_Tresh),4);break;
            case GREEN_LASER:G_Tresh=(int)(short)req->wValue;if(G_Tresh<0)G_Tresh=0;else if(G_Tresh>100)G_Tresh=100;
              ReloadPWMColors();USBD_CtlSendData (pdev,(uint8_t*)(&G_Tresh),4);break;
            case BLUE_LASER:B_Tresh=(int)(short)req->wValue;if(B_Tresh<0)B_Tresh=0;else if(B_Tresh>100)B_Tresh=100;
              ReloadPWMColors();USBD_CtlSendData (pdev,(uint8_t*)(&B_Tresh),4);break;
            default:Cmd=NO_CMD;return USBD_FAIL;//неправильные данные
            }
            Cmd=NO_CMD;
            return USBD_OK;
          }
         //---------------------------------------
         
          default://неопределенная команда,нельзя обработать
            Cmd=NO_CMD;
            return USBD_FAIL;  
					
         } //switch cmd
        }//if
      //=============================================================
        else //направление - от хоста к устройству (OUT DIRECTION)
        {
          // Set the value of the current command to be processed 
          Cmd = req->bRequest;
          CmdLen = req->wLength;
          
          //---------------------------------------
          switch(Cmd)
          {
            //_________________________________LSCAN__________________________________________________
	    case SET_XY://вывод x,y через командный поток
            _setDACs(req->wValue,req->wIndex);
            Cmd=NO_CMD;
            return USBD_OK;
           //---------------------------------------
           case OPEN_SEC_WND://открыть окно безопасности
             //OpenSecWindow((uint8_t)req->wValue);
             break;
           //---------------------------------------
           case CLOSE_SEC_WND://закрыть окно безопасности
             //CloseSecWindow((uint8_t)req->wValue);
             break;
           //===================  
          
           default://неопределенная команда,нельзя обработать
            Cmd=NO_CMD;
            return USBD_FAIL;
          } //switch cmd
                    
        }//else
     
  //.......   
  default://неопределенный тип запроса req->bmRequest ,нельзя обработать
     Cmd=NO_CMD;
     return USBD_FAIL;
  }//switch (req->bmRequest & USB_REQ_TYPE_MASK)
  //--------------------------------------------------------		 
}

//------------------------------------------------------------------------------


/**
  * @brief  LScanCmd
  *         Manage the vendor class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
uint16_t LScan_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len)//обработчик командного буфера
{ 
  if (Cmd == NO_CMD)return USBD_OK;
  
  //---------------------------------------
  switch(Cmd)
  {
   
   default:
    // обработка данных в inter_layer
    APP_FOPS.pIf_Ctrl(Cmd, CmdBuff, CmdLen);//передать в inter_layer(там обработчик)
		break;
  }
    
  // Reset the command variable to default value 
  Cmd = NO_CMD;
  return USBD_OK;
}

//------------------------------------------------------------------------------
uint8_t  LScanStreamCmd(uint8_t* pbuf,uint8_t cmd)
{
    switch(cmd)
    {
      //---------------LScan----------------------
      case LSCAN_CMD_PLAY://данные уже в буфере
      break;
      
      case LSCAN_CMD_PAUSE://
      break;
				
      case LSCAN_CMD_STOP:
	    break;		
	//------неизвестная команда--------------
			
      default:
      // обработка данных в inter_layer
    	//return APP_FOPS.pIf_Ctrl(Cmd, CmdBuff, CmdLen);//передать в inter_layer(там обработчик)
			
			break;
    }
  return USBD_OK;
}
/* ---------------------------------------------------------------------------*/
/******************* (C) COPYRIGHT 2014 dem1305 *****END OF FILE***************/