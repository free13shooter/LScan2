/**
  Laser project by dem1305
  �������� ������ LScan
  @2014
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSCAN_PROTOCOL_H
#define __LSCAN_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "LScan.h"

/* ---------------------------------------------------------------------------*/
typedef enum
{
  LSCAN_CMD_STOP=0,
  LSCAN_CMD_PLAY,
  LSCAN_CMD_PAUSE,
}LSCAN_CMD_TypeDef;

#define	STATUS_STOP     ((uint8_t)0)
#define	STATUS_PLAY     ((uint8_t)1)
#define	STATUS_PAUSE    ((uint8_t)2)

//-------------------LSCAN COMMANDS---------------------------
//�������, EP0 OUT
#define SET_XY							 16   //����� x,y ����� ��������� �����
#define OPEN_SEC_WND         17   //������� ���� ������������
#define CLOSE_SEC_WND        18   //������� ���� ������������

//�������, EP0 IN
#define SET_GET_SHIFT_K	          32	//������ ����������� ������� ������ � �������� ����� �������� ��� �������� 
#define SET_GET_LINEARITY	  39    //������ �������� ���������� ������ � �������� ����� �������� ��� �������� 
#define SET_GET_COLOR_FREQ        40    //������ � �������� �������� ������� ��������� �������
#define GET_LS_DISCR              41    //�������� ������� ������������� ������,2 ����� (ushort) � ������� ����� � ������ � ������ 2 ����� (ushort)
#define SET_GET_COLOR_SHIFT	  42    //������ �������� ����� � �������� ����� �������� ��� �������� 
#define SET_GET_LASER_POWER	  43    //������ ������� ������ � �������� ����� �������� ��� �������� 
#define SET_GET_PWM_LIMIT	  44    //������ ������� �������� � �������� ����� �������� ��� ��������  
#define GET_STR_BUF_SIZE	  45    //�������� ������ ���������� ������  � ������
#define SET_GET_PWM_TRESH	  46    //������ ����� �������� � �������� ����� �������� ��� ��������
#define SET_GET_TTL_MOD		  47    //������ ����� ��������� TTL/����������

//��������� ����-��������� � ��������� ���������
#define encOnRGBshift	  1
#define encOffRGBshift    2

#define encOnRGBshift_k	  1
#define encOffRGBshift_k  2

#define encOnR	  1
#define encOffR   2
#define encOnG	  3
#define encOffG   4
#define encOnB	  5
#define encOffB   6
#define encRmin	  1
#define encRmax   2
#define encGmin	  3
#define encGmax   4
#define encBmin	  5
#define encBmax   6

//������ ��������������� �� ������:
//����������� ������: 8 ����,������� ���������
#define PATTERN_SIZE         8

/* ---------------------------------------------------------------------------*/
uint8_t  LScanStreamCmd(uint8_t* pbuf,uint8_t cmd);
uint16_t LScanCmd (void  *pdev,USB_SETUP_REQ *req);//���������� ������ OUT EP0 LScan
/* ������ �������� �� �������� ����� ���������� EP0_OUT (RX)*/
uint16_t LScan_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len);//���������� ���������� ������
/* ---------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
#endif /* __LSCAN_PROTOCOL_H */
/******************* (C) COPYRIGHT 2014 dem1305 *****END OF FILE****/
