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
#define _MP(_x,_y)        ((char)128),((char)(_x)),((char)(_y)) //set position(lasers off)(относительно текущей)
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

//=========тип шрифта===========
typedef enum{
GUI_FONTTYPE_PROP=0,//пропорциональный 
GUI_FONTTYPE_MONO //моноширинный
}GUI_FONTTYPE;
//=========описание символа==========
typedef struct {
  unsigned char XSize;//ширина символа в битах
  unsigned char XDist;//дистанция до следующего символа в битах от начала символа
  unsigned char BytesPerLine;//ширина битовой маски в байтах для линии растра
  const unsigned char * pData;//указатель на битовый набор
} GUI_CHARINFO;
//=========свойства шрифта===========
typedef struct GUI_FONT_PROP {
  unsigned short First;                                //первый символ
  unsigned short Last;                                 //первый символ
  const GUI_CHARINFO * paCharInfo;           //адрес первого символа
  const struct GUI_FONT_PROP * pNext;        //указатель на следующую GUI_FONT_PROP
} GUI_FONT_PROP;
//=========шрифт===========
typedef struct {
 unsigned char FType; //тип шрифта - пропорциональный
 unsigned char YSize; //высота шрифта
 unsigned char YDist; //пространство по оси y
 unsigned char XMag; // увеличение по x
 unsigned char YMag; // увеличение по y
 GUI_FONT_PROP* pGUI_FONT_PROP ;}GUI_FONT;
//=========================
typedef struct GUI_FONT_INFO {
  unsigned short First;                        /* first character               */
  unsigned short Last;                         /* last character                */
  const GUI_CHARINFO* paCharInfo;    /* address of first character    */
  const struct GUI_FONT_INFO* pNext; /* pointer to next */
} GUI_FONT_INFO;

//----------------------------------------------------------------------------------
//вывод текста с текущей дискретизацией в позиции и с размером RECT.возврат количества выведенных символов
UINT OutText(RECT* rps,TCHAR* ptx,DWORD color);
//вывод знака в позиции SPOINT,если не NULL,иначе в текущей,с заданным цветом.scale-множитель размера.
void OutVectorChar(unsigned short chIndex,RECT* rpos_size,DWORD SymColor,int loops,uint8_t discrMS,bool vector_mode );
bool searchPoint(int H,int V,SPOINT* A,bool around);
bool searchPointFast(int H,int V,SPOINT* A,bool around);
bool getVector(int H,int V,SPOINT* A);
//сканирование и вывод матричного изображения.vector_mode-флаг прорисовки сплошным(не растр)вектором
void OutMatrix(int H,int V,RECT* rpos_size,DWORD SymColor,uint8_t discr,bool vector_mode );
//запись в массив и вывод в поток при заполнении или принудительно
bool pushToStreamArray(SPOINT* arr,int len,uint8_t discr,int* cnt,uint8_t* pts_pack,SPOINT* pt);
bool startStreamArray(SPOINT* arr,int len,uint8_t discr,int* cnt,uint8_t* pts_pack);//принудительный вывод
//оптимизация вектора для текущей дискретизации.Вернет true,если необходимо вставить точку I
bool OptimizeVector(SPOINT* A,SPOINT* B,SPOINT* I, uint8_t discr);
//----------------------------------------------------------------------------------
/******************* (C) COPYRIGHT 2015 dem1305 *****END OF FILE****/
#endif //__FONT_FUNC_H