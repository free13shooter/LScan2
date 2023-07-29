/**
  ******************************************************************************
  * @file    font_func.h 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    3-okt-2015
  * @brief   font vector drawing functions
  ******************************************************************************
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FONT_FUNC_H
#define __FONT_FUNC_H

/* Includes ------------------------------------------------------------------*/
#include "LScan_types.h"
#include "font_def.h"
#include "ff.h"
#include "ILDA_player.h"
//----------------------------------------------------------------------------------
#define I8    signed char
#define U8  unsigned char     /* unsigned 8  bits. */
#define I16   signed short    /*   signed 16 bits. */
#define U16 unsigned short    /* unsigned 16 bits. */
#define I32   signed long   /*   signed 32 bits. */
#define U32 unsigned long   /* unsigned 32 bits. */
#define I16P I16              /*   signed 16 bits OR MORE ! */
#define U16P U16              /* unsigned 16 bits OR MORE ! */

//macro movement of vector
#define _MP(_x,_y)        ((char)128),((char)(_x)),((char)(_y)) //set position(lasers off)(������������ �������)
//lasers on,move pen
#define _MX(_x)           ((char)64),((char)(_x)),((char)0)//left|right
#define _MY(_y)           ((char)32),((char)0),((char)(_y))//up|down
#define _MV(_x,_y)        ((char)96),((char)(_x)),((char)(_y))//x+y

/*
      ****************************************
      *                                      *
      *      FONT info structure             *
      *                                      *
      ****************************************

This structure is used when retrieving information about a font.
It is designed for future expansion without incompatibilities.
*/
typedef struct {
  unsigned short Flags;
  unsigned char Baseline;
  unsigned char LHeight;     /* height of a small lower case character (a,x) */
  unsigned char CHeight;     /* height of a small upper case character (A,X) */
} GUI_FONTINFO;

#define GUI_FONTINFO_FLAG_PROP (1<<0)    /* Is proportional */
#define GUI_FONTINFO_FLAG_MONO (1<<1)    /* Is monospaced */
#define GUI_FONTINFO_FLAG_AA   (1<<2)    /* Is an antialiased font */
#define GUI_FONTINFO_FLAG_AA2  (1<<3)    /* Is an antialiased font, 2bpp */
#define GUI_FONTINFO_FLAG_AA4  (1<<4)    /* Is an antialiased font, 4bpp */

//=========��� ������===========
typedef enum{
GUI_FONTTYPE_PROP=0,//���������������� 
GUI_FONTTYPE_MONO //������������
}GUI_FONTTYPE;
//=========�������� �������==========
typedef struct {
  unsigned char XSize;//������ ������� � �����
  unsigned char XDist;//��������� �� ���������� ������� � ����� �� ������ �������
  unsigned char BytesPerLine;//������ ������� ����� � ������ ��� ����� ������
  const unsigned char * pData;//��������� �� ������� �����
} GUI_CHARINFO;
//=========�������� ������===========
typedef struct GUI_FONT_PROP {
  unsigned short First;                                //������ ������
  unsigned short Last;                                 //������ ������
  const GUI_CHARINFO * paCharInfo;           //����� ������� �������
  const struct GUI_FONT_PROP * pNext;        //��������� �� ��������� GUI_FONT_PROP
} GUI_FONT_PROP;
//=========�����===========
typedef struct {
 unsigned char FType; //��� ������ - ����������������
 unsigned char YSize; //������ ������
 unsigned char YDist; //������������ �� ��� y
 unsigned char XMag; // ���������� �� x
 unsigned char YMag; // ���������� �� y
 GUI_FONT_PROP* pGUI_FONT_PROP ;}GUI_FONT;
//=========================
typedef struct GUI_FONT_INFO {
  unsigned short First;                        /* first character               */
  unsigned short Last;                         /* last character                */
  const GUI_CHARINFO* paCharInfo;    /* address of first character    */
  const struct GUI_FONT_INFO* pNext; /* pointer to next */
} GUI_FONT_INFO;

//----------------------------------------------------------------------------------
//����� ������ � ������� �������������� � ������� � � �������� RECT.������� ���������� ���������� ��������
UINT OutText(RECT* rps,TCHAR* ptx,DWORD color);
//����� ����� � ������� SPOINT,���� �� NULL,����� � �������,� �������� ������.scale-��������� �������.
void OutVectorChar(unsigned short chIndex,RECT* rpos_size,DWORD SymColor,int loops,uint8_t discrMS,bool vector_mode );
bool searchPoint(int H,int V,SPOINT* A,bool around);
bool searchPointFast(int H,int V,SPOINT* A,bool around);
bool getVector(int H,int V,SPOINT* A);
//������������ � ����� ���������� �����������.vector_mode-���� ���������� ��������(�� �����)��������
void OutMatrix(int H,int V,RECT* rpos_size,DWORD SymColor,uint8_t discr,bool vector_mode );
//������ � ������ � ����� � ����� ��� ���������� ��� �������������
bool pushToStreamArray(SPOINT* arr,int len,uint8_t discr,int* cnt,uint8_t* pts_pack,SPOINT* pt);
bool startStreamArray(SPOINT* arr,int len,uint8_t discr,int* cnt,uint8_t* pts_pack);//�������������� �����
//����������� ������� ��� ������� �������������.������ true,���� ���������� �������� ����� I
bool OptimizeVector(SPOINT* A,SPOINT* B,SPOINT* I, uint8_t discr);
//----------------------------------------------------------------------------------
/******************* (C) COPYRIGHT 2015 dem1305 *****END OF FILE****/
#endif //__FONT_FUNC_H