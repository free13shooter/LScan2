/**
  Laser project by dem1305

  @2014
*/
#include "LScan_player.h"
/* Defines -------------------------------------------------------------------*/
/*
typedef struct {
    char		headerSym[4];// �I�, �L�, �D�, �A�  The ASCII letters ILDA, identifying an ILDA format header. 
    uint8_t		zBytes[3];//��� ����� ����� . �� ������������, �� ������ ���� �������.
    uint8_t		format;   // ������-��� 2D(3D) ��������; 0 ��� 3-D ��� ��������� 1 ��� 2-D. � 3-D ������ ������ ����� �� ����� � � 2-D ������ ��� ����� �� �����.
    char		frameName[8];//������ �������� ASCII � ������ ����� �����.
    char		company[8]; 	//������ �������� ASCII � ������ ��������, ������� ������� ����(�����). 
    uint16_t 	pointsTotal;//���������� ����� � ������� �����(0-65535), ���� 0,����� ��� ����������� �� ����� �����(���������) � ������ ������ �� �����.
    uint16_t 	frameNum;  //���� ����� - ����� ������ ����� ��� ������������������ ��������, ��� ������������ ����� �����. ������� ���������� � ������ 0. �������� ������ 0-65535.
    uint16_t 	framesTotal;//����� ������� � ���� ������ ��� ������������������.�������� 1-65535. 
    uint8_t		scannerNum; //����� ������ ������� ��� ���������.�������� 0-255. ��� LScan ����� 0.
    uint8_t 	future;    //�������������� ��� �������� �������������. ������ ���� ���������� � 0.
} ilda_hdr;

//������������ ������ (�������� ������ �  ��� ������� ������)

����� �� ���������� ������� ������ � ����������� ����� � ��������� �������: 
[X ����������][Y ����������][Z ����������][Status Code] 
���������� �� ������ ��� �������� �� 2 �����, ������-��� ����� ���������� � ����� ��������� ������� � � ����� �������� �����. 
�������� �������(Format Code=2) ����� ������� ���������, �� ������ ��������� ����� �������� ���������� �� ������������ ������ � ������� RGB.
��� TrueColor ������(Format Code=3) �������� ����������� ���������� �������� �������� � ������� RGB.

X:16-��������� �������� �������� �����. ������� ����� -32768; ������������� �������� ����� +32767. 
(��� ����������� ���������� ��� ������������� ����������� ��������.) 
Y:������������� ������ �����-32768; ������������� ������� +32767. 
Z:16-��������� �������� �������� �����.  ������������� ������ ����� (������ �� ���������� �� �����;������ ������),-32768;
������������� �������� ������� (� ���������� �� �����;; ����� �������),+32767. ��� ��� ����� �� ����������� ���� 
format - ��� (���� 8) ��������� �� 2-D - ��������� �����. 

state:
-���� ���� ��������� 0-7 (lsb) ��������� �� ����� ����� ����� �� ������� ������, ���������� �������, ������� � ����� ��������.
  ��. ������� ������ ILDA.������ ��������� ��� �������������� ����������. 
-���� 8-13 �� ������������ � ������ ���� ����������� � 0 (�����������������). 
-��� 14 �������� ����� ������� (blanking). ���� ��� - 0, �� ����� �������. ���� 1, �� ����� ��������.
  ��� ������� ���������� ���� ���, ���� ���� ������������ ������� ���������� ������ ���� 0-7 ��� �������/������. 
-��� 15 (msb) �������� ��� ���������� �����. ���� ��� ���������� � 0 ��� ���� ����� ����� ���������. 1 ��������� �� ����� ������ ��������.
  ��� ������� ��� ������������� � ������������� ������������� ���������; �������� ��� ���� � ������ 25-26 ((����� ���������� �����) - 
  ����������� ������� ����� �����. 

������� ������ - HIGH ����, �������������� ������� ������. ��������,��� ����� 11111111 00000000 ������������ ���������� ����� 65280,�� 255.

���� ��������� ��������� ����� ������� �������� ��� ������ ���������� ����� (����� 25-26), ����� ��� - ��������� ��������� � ����� � 
���� ����� ���� ������. 
*/

typedef struct coord2d {
	  uint16_t synchroW;//����� ������������� = 0
    int16_t x;
    int16_t y;
    uint8_t r;
		uint8_t g;
		uint8_t b;
} coord2d;

typedef struct coord3d {
    uint16_t synchroW;//����� ������������� = 1
    int16_t x;
    int16_t y;
    uint8_t r;
		uint8_t g;
		uint8_t b;
} coord3d;





/*----------------------------------------------------------------------------*/

/******************* (C) COPYRIGHT 2014 dem1305 *****END OF FILE****/