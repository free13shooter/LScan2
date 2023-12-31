/**
  Laser project by dem1305

  @2014
*/


/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __ILDA_PLAYER_H
#define __ILDA_PLAYER_H

/* ---------------------------------------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------------------------------------*/
#include "LScan.h"
#include "LScan_types.h"
#include "ff.h"

/* ---------------------------------------------------------------------------------------------------------*/
//function prototypes

//void Init_Player();
//void DeInit_Player();
IS_RES Proccess_Player();//����� ILDA � ������ ������������,��������� ��������� ������.
IS_RES playILDAFile(TCHAR* filename);


void SetXYPos(int x,int y);//����������� ������� ���������� (���� �������)
void GetDAC(int* x,int* y,bool ILDA_format);//�������� ������� �������� ��������� � ILDA/uint16_t ���������
//����� � ��������� �����,wait_num - ������ �������� ���� ������������ ����� � ������
bool insert_SPOINT_to_stream(SPOINT* pt,uint8_t discr_in_MS,uint8_t points_in_sample,uint32_t wait_num);
/* ---------------------------------------------------------------------------------------------------------*/
//���������� �� ������� � ����������� �� n ������������������ ,n>1-�������� � �2
#define _crd(c1,c2,n)           (((c1)+(n)*(c2))/(1.0f+(n)))  

/* ---------------------------------------------------------------------------------------------------------*/
#endif /* __ILDA_PLAYER_H */
/******************* (C) COPYRIGHT 2015 dem1305 *****END OF FILE****/
