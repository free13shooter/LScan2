/**
  Laser project by dem1305
  �������� ������ LScan,USB
  @2014
*/


/* Includes ------------------------------------------------------------------*/
#include "LScan_protocol.h"    //�������� ������ LScan ,usb ������� �������

/* ---------------------------------------------------------------------------*/


/* Defines ------------------------------------------------------------------*/

/* Exports ------------------------------------------------------------------*/
extern CUSTOM_IF_Prop_TypeDef IF_cbfs_fops;

extern uint8_t USB_IsoRx_Buffer [] ;//����� ����������� ������
extern uint16_t USB_IsoLScanDSSsize_Buffer [];//��������������� ����� LScan ��� �������� ������� ������ DSS(�������������)

extern uint8_t CmdBuff [] ;					//��������� �����,�������� 1��,������������ ����������� ��������� ����� 0x00/0x80
extern uint8_t USB_Rx_Buffer [] ;		//����� ��������� ������
extern uint8_t* IntOutWrPtr;
extern uint8_t* IntOutRdPtr;

extern uint8_t APP_Tx_Buffer [] ;		//����� ��������� ������� 
extern uint32_t APP_Tx_ptr_in ;			//����� �����
extern uint32_t APP_Tx_ptr_out ;		//����� ������
extern uint8_t DisableTX;						//���� ���������� �������� ����� EP INT IN(������������� �������)
extern uint16_t IsocOutPacketSize;  //

extern uint32_t Cmd;   //core
extern uint32_t CmdLen;//core

//������ ��������������� �� ������ USB_IsoRx_Buffer: ����������� ������: 9 ����,������� ���������
extern RCC_ClocksTypeDef RCC_Clocks;
//extern int PlayDiscrFreq;//������� �����

uint8_t* ptrByte; //��������������� ���������
uint16_t* ptrWord;//��������������� ���������
uint32_t* ptrDWord;//��������������� ���������

extern uint8_t LScan_CMD;

uint8_t LScanStatus=STATUS_STOP;

extern uint8_t* BEG;	//������ ���� ������
extern uint8_t* END;	//��������� ���� ������

extern uint16_t SampleDiscrInMs;//������� ������������� ��� 1 ��-������
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
  
//---------- ���������� ---------------------------
//��.usbd_core.c,usbd_ioreq.c
/*
typedef  struct  usb_setup_req {
    
    uint8_t   bmRequest;  //��� �������(�������� � usbd_core.c)=VENDOR                    
    uint8_t   bRequest;   //��� �������                          
    uint16_t  wValue;     //���.��������                              
    uint16_t  wIndex;     //������                             
    uint16_t  wLength;    //����� ������ ����������� ��/���������� ����� ������                            
} USB_SETUP_REQ;
*/
uint16_t LScanCmd (void  *pdev,USB_SETUP_REQ *req)//���������� ������ OUT EP0 LScan
{
  Cmd=req->bRequest;
  uint16_t buf[2];
  uint32_t val32=0;
  uint16_t sampleBytes=(uint16_t)(_bytesFromTo(IsocOutRdPtr,IsocOutWrPtr,LS_BUF_SIZE));
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)//&0x60
  {
  //.......
  case USB_REQ_TYPE_VENDOR : 
      if (req->bmRequest & 0x80)//����������� - �� ���������� � �����(IN DIRECTION)
        {
          Cmd = req->bRequest;
          //---------------------------------------
          //������ �������������� �����������
          if(Cmd>=(uint8_t)((uint8_t)SET_GET_SHIFT_K|(uint8_t)encOnR)&&
             Cmd<=(uint8_t)((uint8_t)SET_GET_SHIFT_K|(uint8_t)encOffB))
          {
            val32=((uint32_t)req->wValue|((uint32_t)req->wIndex<<16));
            
            switch(Cmd&=(uint8_t)~SET_GET_SHIFT_K)
            {
              case encOnRGBshift_k:memcpy(&RGB_on_k,&val32,4);USBD_CtlSendData (pdev,(uint8_t*)(&RGB_on_k),4);break;
              case encOffRGBshift_k:memcpy(&RGB_off_k,&val32,4);USBD_CtlSendData (pdev,(uint8_t*)(&RGB_off_k),4);break;
              default:Cmd=NO_CMD;return USBD_FAIL;//������������ ������
            }
            Calculate_Colors_Shifting_and_Sampling();
            Cmd=NO_CMD;
            return USBD_OK;
          }
         //---------------------------------------
         switch(Cmd)
         {
          //_________________________________LSCAN__________________________________________________
	  case SET_GET_LINEARITY: //������ ������� �������� ������[1.01;100] (1.01-�������� �������) � �������� ����� �������� ��� ��������
            val32=((uint32_t)req->wValue|((uint32_t)req->wIndex<<16));
            memcpy((void*)(&Linearity),&val32,4);USBD_CtlSendData (pdev,(uint8_t*)(&Linearity),4);
            Calculate_Colors_Shifting_and_Sampling();
            return USBD_OK; 
          //---------------------------------------
          case GET_LS_DISCR: //�������� ������� ������������� ���������,2 ����� discr � 2 ����� ���������� ������ � ������
            buf[0]=(uint16_t)SampleDiscrInMs;//SampleDiscrInMs;//������� ������������� ��� 1 ��-������
            buf[1]=sampleBytes;
            //EP0_TX(pdev,(uint8_t*)buf,4);//4 �����(int)
            USBD_CtlSendData(pdev,(uint8_t*)buf,4);//4 �����(int)
            Cmd=NO_CMD;
            return USBD_OK; 
          //---------------------------------------
          case GET_STR_BUF_SIZE:   //�������� ������ ���������� ������  � ������
            buf[0]=(uint16_t)(ISOC_OUT_PACKET_SIZE+ISO_Buff_Size);
            USBD_CtlSendData (pdev,(uint8_t*)&buf[0],2);//2 �����(uint16_t)
            Cmd=NO_CMD;
            return USBD_OK; 
          //---------------------------------------
          case SET_GET_TTL_MOD://������ ����� ��������� TTL/���������� � �������� ����� �������� ��� �������� 
            TTL_modulation=(uint8_t)(req->wValue);
            USBD_CtlSendData (pdev,(uint8_t*)(&TTL_modulation),1);
            Cmd=NO_CMD;
            return USBD_OK;
          //---------------------------------------
	  case SET_GET_COLOR_SHIFT://������ ����������� ������� ������ � �������� ����� �������� ��� �������� 
            switch(req->wIndex)
            {
              case encOnRGBshift:RGB_on_shift=(int)(req->wValue)*1000;USBD_CtlSendData (pdev,(uint8_t*)(&RGB_on_shift),4);break;
              case encOffRGBshift:RGB_off_shift=(int)(req->wValue)*1000;USBD_CtlSendData (pdev,(uint8_t*)(&RGB_off_shift),4);break;
              default:Cmd=NO_CMD;return USBD_FAIL;//������������ ������
            }
            Calculate_Colors_Shifting_and_Sampling();
            Cmd=NO_CMD;
            return USBD_OK;
          //---------------------------------------
          case SET_GET_PWM_LIMIT://������ ������� �������� � �������� ����� �������� ��� �������� 
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
              default:Cmd=NO_CMD;return USBD_FAIL;//������������ ������
            }
            Cmd=NO_CMD;
            return USBD_OK;
          //---------------------------------------
          case SET_GET_LASER_POWER://������ ������� ������ � �������� ����� �������� ��� �������� 
            switch(req->wIndex)
            {
              case RED_LASER:R_power=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&R_power),4);
              ReloadPWMColors();break;
              case GREEN_LASER:G_power=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&G_power),4);
              ReloadPWMColors();break;
              case BLUE_LASER:B_power=(int)(short)req->wValue;USBD_CtlSendData (pdev,(uint8_t*)(&B_power),4);
              ReloadPWMColors();break;
              default:Cmd=NO_CMD;return USBD_FAIL;//������������ ������
            }
            Cmd=NO_CMD;
            return USBD_OK; 
	  //---------------------------------------
	  case SET_GET_COLOR_FREQ://������ � �������� �������� ������� ��������� �������
          {
            //int
            Colors_PWM_Freq=(int)(short)req->wValue;if(Colors_PWM_Freq<1)Colors_PWM_Freq=1;else if(Colors_PWM_Freq>168)Colors_PWM_Freq=168;
            USBD_CtlSendData (pdev,(uint8_t*)(&Colors_PWM_Freq),4);
            ReloadPWMColors();
            return USBD_OK;
          }
         //---------------------------------------
          case SET_GET_PWM_TRESH://������ ����� �������� � �������� ����� �������� ��� ��������
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
            default:Cmd=NO_CMD;return USBD_FAIL;//������������ ������
            }
            Cmd=NO_CMD;
            return USBD_OK;
          }
         //---------------------------------------
         
          default://�������������� �������,������ ����������
            Cmd=NO_CMD;
            return USBD_FAIL;  
					
         } //switch cmd
        }//if
      //=============================================================
        else //����������� - �� ����� � ���������� (OUT DIRECTION)
        {
          // Set the value of the current command to be processed 
          Cmd = req->bRequest;
          CmdLen = req->wLength;
          
          //---------------------------------------
          switch(Cmd)
          {
            //_________________________________LSCAN__________________________________________________
	    case SET_XY://����� x,y ����� ��������� �����
            _setDACs(req->wValue,req->wIndex);
            Cmd=NO_CMD;
            return USBD_OK;
           //---------------------------------------
           case OPEN_SEC_WND://������� ���� ������������
             //OpenSecWindow((uint8_t)req->wValue);
             break;
           //---------------------------------------
           case CLOSE_SEC_WND://������� ���� ������������
             //CloseSecWindow((uint8_t)req->wValue);
             break;
           //===================  
          
           default://�������������� �������,������ ����������
            Cmd=NO_CMD;
            return USBD_FAIL;
          } //switch cmd
                    
        }//else
     
  //.......   
  default://�������������� ��� ������� req->bmRequest ,������ ����������
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
uint16_t LScan_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len)//���������� ���������� ������
{ 
  if (Cmd == NO_CMD)return USBD_OK;
  
  //---------------------------------------
  switch(Cmd)
  {
   
   default:
    // ��������� ������ � inter_layer
    APP_FOPS.pIf_Ctrl(Cmd, CmdBuff, CmdLen);//�������� � inter_layer(��� ����������)
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
      case LSCAN_CMD_PLAY://������ ��� � ������
      break;
      
      case LSCAN_CMD_PAUSE://
      break;
				
      case LSCAN_CMD_STOP:
	    break;		
	//------����������� �������--------------
			
      default:
      // ��������� ������ � inter_layer
    	//return APP_FOPS.pIf_Ctrl(Cmd, CmdBuff, CmdLen);//�������� � inter_layer(��� ����������)
			
			break;
    }
  return USBD_OK;
}
/* ---------------------------------------------------------------------------*/
/******************* (C) COPYRIGHT 2014 dem1305 *****END OF FILE***************/