/**
  Laser project by dem1305

  @2014
*/


/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __LSCAN_TYPES_H
#define __LSCAN_TYPES_H

//----------------------------------------------------------------------------//
#include "integer.h" //типы
#include <stdint.h>
#include <string.h> //mem... in DLib_Product
//----------------------------------------------------------------------------//
#define VOID void
typedef char CHAR;

#define MAX_UINT32T 0xFFFFFFFF
#define MAX_UINT    MAX_UINT32T
#define HALF_UINT32_MAX 2147483647
/* This type MUST be 64-bit (Remove this for C89 compatibility) */
typedef unsigned long long QWORD;
/* This type MUST be 8 bit */
//typedef unsigned char bool ;
//----------------------------------------------------------------------------//

//#define true ((unsigned char)1)
//#define false ((unsigned char)0)
typedef unsigned char	UCHAR;
/* This type MUST be 16 bit */
typedef unsigned short	USHORT;
/* This type MUST be 32 bit */
typedef unsigned long	ULONG;
//----------------------------------------------------------------------------//
#define MAXLONG             uint32_t

#define far
#define near

#ifndef CONST
#define CONST               const
#endif
/* These types MUST be 32-bit */
typedef long		    LONG;
typedef unsigned long	    DWORD;

typedef int                 BOOL;
/* This type MUST be 8-bit */
typedef unsigned char	    BYTE;
/* These types MUST be 16-bit */
typedef short		    SHORT;
typedef unsigned short	    WORD;
typedef unsigned short	    WCHAR;

typedef float               FLOAT;
typedef FLOAT               *PFLOAT;
typedef BOOL near           *PBOOL;
typedef BOOL far            *LPBOOL;
typedef BYTE near           *PBYTE;
typedef BYTE far            *LPBYTE;
typedef int near            *PINT;
typedef int far             *LPINT;
typedef WORD near           *PWORD;
typedef WORD far            *LPWORD;
typedef long far            *LPLONG;
typedef DWORD near          *PDWORD;
typedef DWORD far           *LPDWORD;
typedef void                *PVOID;
typedef void far            *LPVOID;
typedef CONST void far      *LPCVOID;
/* These types MUST be 16-bit or 32-bit */
typedef int                 INT;
typedef unsigned int        UINT;
typedef unsigned int        *PUINT;


#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif

//----------------------------------------------------------------------------//
#ifndef _CHAR_W_UNI_
#define _CHAR_W_UNI_

typedef WCHAR *PWCHAR, *LPWCH, *PWCH;
typedef CONST WCHAR *LPCWCH, *PCWCH;

#define _Null_terminated_ 
#define _NullNull_terminated_
#define UNALIGNED
#define _In_      //input 
#define _In_opt_  //optional input

typedef _Null_terminated_ WCHAR *NWPSTR, *LPWSTR, *PWSTR;
typedef _Null_terminated_ PWSTR *PZPWSTR;
typedef _Null_terminated_ CONST PWSTR *PCZPWSTR;
typedef _Null_terminated_ WCHAR UNALIGNED *LPUWSTR, *PUWSTR;
typedef _Null_terminated_ CONST WCHAR *LPCWSTR, *PCWSTR;
typedef _Null_terminated_ PCWSTR *PZPCWSTR;
typedef _Null_terminated_ CONST WCHAR UNALIGNED *LPCUWSTR, *PCUWSTR;

typedef _NullNull_terminated_ WCHAR *PZZWSTR;
typedef _NullNull_terminated_ CONST WCHAR *PCZZWSTR;
typedef _NullNull_terminated_ WCHAR UNALIGNED *PUZZWSTR;
typedef _NullNull_terminated_ CONST WCHAR UNALIGNED *PCUZZWSTR;

typedef  WCHAR *PNZWCH;
typedef  CONST WCHAR *PCNZWCH;
typedef  WCHAR UNALIGNED *PUNZWCH;
typedef  CONST WCHAR UNALIGNED *PCUNZWCH;
#endif
//----------------------------------------------------------------------------//
//
// ANSI (Multi-byte Character) types
//
typedef CHAR *PCHAR, *LPCH, *PCH;
typedef CONST CHAR *LPCCH, *PCCH;

typedef _Null_terminated_ CHAR *NPSTR, *LPSTR, *PSTR;
typedef _Null_terminated_ PSTR *PZPSTR;
typedef _Null_terminated_ CONST PSTR *PCZPSTR;
typedef _Null_terminated_ CONST CHAR *LPCSTR, *PCSTR;
typedef _Null_terminated_ PCSTR *PZPCSTR;

typedef _NullNull_terminated_ CHAR *PZZSTR;
typedef _NullNull_terminated_ CONST CHAR *PCZZSTR;

typedef  CHAR *PNZCH;
typedef  CONST CHAR *PCNZCH;
//------------------------------------------------------------------------------
//
// Neutral ANSI/UNICODE types and macros
//
#ifdef  UNICODE                     // r_winnt

#ifndef _TCHAR_DEFINED
typedef WCHAR TCHAR, *PTCHAR;
typedef WCHAR TBYTE , *PTBYTE ;
#define _T(x) L ## x
#define _TEXT(x) L ## x
#define _TCHAR_DEFINED
#endif // !_TCHAR_DEFINED 

typedef LPWCH LPTCH, PTCH;
typedef LPCWCH LPCTCH, PCTCH;
typedef LPWSTR PTSTR, LPTSTR;
typedef LPCWSTR PCTSTR, LPCTSTR;
typedef LPUWSTR PUTSTR, LPUTSTR;
typedef LPCUWSTR PCUTSTR, LPCUTSTR;
typedef LPWSTR LP;
typedef PZZWSTR PZZTSTR;
typedef PCZZWSTR PCZZTSTR;
typedef PUZZWSTR PUZZTSTR;
typedef PCUZZWSTR PCUZZTSTR;
typedef PZPWSTR PZPTSTR;
typedef PNZWCH PNZTCH;
typedef PCNZWCH PCNZTCH;
typedef PUNZWCH PUNZTCH;
typedef PCUNZWCH PCUNZTCH;
#define __TEXT(quote) L##quote      // r_winnt

#else   //<<--UNICODE               // r_winnt
//---------------------------------------------
//ANSI-->>
#ifndef _TCHAR_DEFINED
typedef char TCHAR, *PTCHAR;
typedef unsigned char TBYTE , *PTBYTE ;
#define _T(x) x
#define _TEXT(x) x
#define _TCHAR_DEFINED
#endif /* !_TCHAR_DEFINED */

typedef LPCH LPTCH, PTCH;
typedef LPCCH LPCTCH, PCTCH;
typedef LPSTR PTSTR, LPTSTR, PUTSTR, LPUTSTR;
typedef LPCSTR PCTSTR, LPCTSTR, PCUTSTR, LPCUTSTR;
typedef PZZSTR PZZTSTR, PUZZTSTR;
typedef PCZZSTR PCZZTSTR, PCUZZTSTR;
typedef PZPSTR PZPTSTR;
typedef PNZCH PNZTCH, PUNZTCH;
typedef PCNZCH PCNZTCH, PCUNZTCH;
#define __TEXT(quote) quote         // r_winnt

#endif /* UNICODE/ANSI */                // r_winnt
#define TEXT(quote) __TEXT(quote)   // r_winnt
//------------------------------------------------------------------------------
//char types (UNICODE dependent)
#ifdef UNICODE
  #define LPCHAR  LPWSTR
  #define LPCCHAR LPCWSTR
#else
  #define LPCHAR  LPSTR
  #define LPCCHAR LPCSTR
#endif

//----------------------------------------------------------------------------//
#define INFINITE  ((uint32_t)0xFFFFFFFF) //((ULONG)-1) 4 байта
//----------------------------------------------------------------------------//

//---------FR_EOF--------
#define FR_EOF  FR_NOT_ENABLED //=(12) The volume has no work area */
//----------------------------------------------------------------------------//
//байтов от указателя _fr до указателя _to в кольцевом буфере размером _buf_size
#define DISTRING(_fr,_to,_buf_size) ((_fr)<=(_to)?(int)((_to)-(_fr)):((int)(_buf_size)-(int)(_fr)+(int)(_to)))

#define LOBYTE(w)           ((BYTE)(((WORD)(w)) & 0xff))
#define HIBYTE(w)           ((BYTE)((((WORD)(w)) >> 8) & 0xff))
//----------------------------------------------------------------------------//
//command
typedef enum
{
  IC_NO=0,
  IC_DPSS_ON=1,                 //немедленно включить лазеры DPSS и не использовать коррекцию скорости
  IC_GET_STREAM_PAK_SIZE=128,		//получить размер пакета изохронной точки в 8-байтных единицах
  IC_GET_STREAM_BUF_SIZE=129,		//получить размер потокового буфера в пакетах изохронной точки
  IC_GET_BUF_FREE_PACKS=130,		//получить свободное место в пакетах изохронной точки в потоковом буфере
  IC_SET_USB_DISABLE=131,				//остановить интерфейс USB
  IC_GET_BUF_FREE_256=132,			//получить свободное место в 256-байтных частях
  IC_GET_BUF_FREE_8=133,				//получить свободное место в байтах/8
  IC_GET_LVER=134							  //ПИНГ , вернуть версию LScan (LSCAN_VERSION_NUM)
}ICMD;//intercon cmd

//connection mode , 0=low priority
typedef enum
{
  IM_NO=0,
  IM_WLAN=1,//WIFI intercon
  IM_SD=2,//SD-card
  //IM_USB=3, //USB intercon
  //IM_OTG=4//USB OTG
}IMODE;//current interconnection mode

/*
совместимая с COORDSTR структура минимального трансферного блока. Версия 2
*/
#pragma pack(1)
typedef struct 
{
  uint8_t cmd;    //команда
  uint8_t discrMs;//дискретизация , точек в миллисекунду
  uint8_t lx ;    //младшие байты координат
  uint8_t ly;
  uint8_t hxhy;   //старшие полубайты координат
  uint8_t r;	    //1 байт цвет, 0-255
  uint8_t g;	    //1 байт цвет, 0-255
  uint8_t b;	    //1 байт цвет, 0-255
}ICP;
#pragma pack()
//8 байт 
//----------------------------------------------------------------------------//

//выравнивание 1 байт!!!!!!
#pragma pack(1)
typedef struct {
  UCHAR		headerSym[4];// “I”, “L”, “D”, “A”  The ASCII letters ILDA, identifying an ILDA format header. 
  UCHAR		zBytes[3];   //Три байта нулей . Не используется, но должен быть обнулен.
  UCHAR		format;      // формат-код 2D(3D) анимации; 0 для 3-D или двоичного 1 для 2-D. В 3-D режиме четыре слова на точку и в 2-D режиме три слова на точку.
  UCHAR		frameName[8];//Восемь символов ASCII с именем этого кадра.
  UCHAR		company[8];  //Восемь символов ASCII с именем компании, которая создала кадр(фрейм). 
  USHORT 	pointsTotal; //количество точек в текущем кадре(0-65535), если 0,тогда это принимается за конец файла(заголовка) и больше данных не будет.
  USHORT 	frameNum;    //Если фрейм - часть группы такой как последовательность анимации, это представляет номер кадра. Подсчет начинается с фрейма 0. Диапазон фрейма 0-65535.
  USHORT 	framesTotal; //всего фреймов в этой группе или последовательности.Диапазон 1-65535. 
  UCHAR		scannerNum;  //номер сканера или проектора.Диапазон 0-255. для LScan нужно 0.
  UCHAR 	future;      //Зарезервирован для будущего использования. Должен быть установлен в 0.
} ILDA_COORD_HDR;//32 bytes

typedef struct {
  UCHAR		headerSym[4];// “I”, “L”, “D”, “A”  The ASCII letters ILDA, identifying an ILDA format header. 
  UCHAR		zBytes[3];   //Три байта нулей . Не используется, но должен быть обнулен.
  UCHAR		format;      // формат-код  2 для таблицы цветов
  UCHAR		paletteName[8];//Восемь символов ASCII с именем этой палитры.
  UCHAR		company[8];  //Восемь символов ASCII с именем компании, которая создала кадр(фрейм). 
  USHORT 	colorsTotal; //всего цветов в таблице (от 2)
  USHORT 	paletteNum;    //номер палитры. Подсчет начинается с 0. Диапазон  0-65535.
  USHORT 	future;      //Зарезервирован для будущего использования. Должен быть установлен в 0.
  UCHAR		scannerNum;  //номер сканера или проектора.Диапазон 0-255. для LScan нужно 0.
  UCHAR 	future1;      //Зарезервирован для будущего использования. Должен быть установлен в 0.
} ILDA_COLOR_HDR;//32 bytes

//ILDA
typedef struct{
  short x;
  short y;
  short z;//не используется в 2D
  USHORT status;
} COORD3D;//8 bytes
//ILDA
typedef struct {
  short x;
  short y;
  short z;
  USHORT status1;
  USHORT status2;
} COORD3D_FORMAT4;//8 bytes
//ILDA
typedef struct {
  short x;
  short y;
  USHORT status;
} COORD2D;//6 bytes
//ILDA
typedef struct {
  BYTE r;
  BYTE g;
  BYTE b;
} CLR;//3 bytes

#pragma pack()

//source point
typedef struct {
  int x,y;
  DWORD color;
} SPOINT;

typedef struct{
  int x; //начальная позиция
  int y; //начальная позиция
  int w; //ширина
  int h; //высота
} RECT;

typedef struct //vector movement structure for optimisation
{
  float v0x;
  float v0y;//накопленные скорости по осям
}OPT_V;
//----------------------------------------------------------------------------//
#define LSCAN_VERSION_NUM   2
//INTERNAL STATUS RESULT
typedef   uint8_t IS_RES;

#define   IS_OK     ((uint8_t)0x0)     //0 ==успешно
#define   IS_ERR    ((uint8_t)0x1)     //1 ==ошибка
#define   IS_EFS    ((uint8_t)0x2)     //2 ==ошибка файловой системы
#define   IS_PLAY   ((uint8_t)0x4)     //4 ==проигрывание
#define   IS_EOF    ((uint8_t)0x8)     //8 ==конец файла
#define   IS_EODIR  ((uint8_t)0x10)    //16 ==конец директория
#define   IS_BUSY   ((uint8_t)0x20)    //32==занят
//----------------------------------------------------------------------------//
//координата на отрезке в зависимости от n пропорциональности ,n>1-смещение к с2
#define _crdi(c1,c2,n)     (((c1)+(n)*(c2))/(1+(n)))  
//logxY = log Y / log X -логарифм по основанию x числа Y
#define logx(base,x) (log10(x)/log10(base))
//байтов от указателя _fr до указателя _to в кольцевом буфере размером _buf_size
#define bytesCircularBufferFromTo(_fr,_to,_buf_size) ((_fr)<=(_to)?(int)((_to)-(_fr)):((int)(_buf_size)-((int)(_fr)-(int)(_to))))
//----------------------------------------------------------------------------//
#define ZeroMemory(mem,len) memset((mem),0,(len))
#define Clear(obj) ZeroMemory((obj),sizeof(obj))
#define _MHZ *1000000
//----------------------------------------------------------------------------//
//servo parameters
typedef struct{
  uint16_t min;   //левое крайнее положение, CLK
  uint16_t max;   //правое крайнее положение, CLK
  uint16_t angle; //диапазон вращения в градусах
} SERVO_PAR;

//PWM parameters
typedef struct{
  uint32_t clk_sec;//тактирование, тактов в секунду
  uint16_t prescaler;//предделитель
  uint16_t period;//длина шим
} PWM_PAR;


//тип воспроизведимых файлов и режим наложения эффектов при воспроизведении.
typedef uint16_t TYPEMODEMASK;

//какие файлы воспроизводить с диска 
#define TM_FILETYPE_BIT         0       //номер бита
#define TM_FILETYPEMASK         1       //маска режима воспроизведения ILDA/MP3
#define TM_ILDA                 0       //обычный режим воспроизведения ILDA
#define TM_MP3                  1       //режим MP3
//как выбирать файлы с диска
#define TM_FILEORDER_BIT        1       //номер бита
#define TM_FILEORDERMASK        2       //маска порядка воспроизведения
#define TM_FILE_ORDER           0       //по порядку
#define TM_FILE_RANDOM          2       //случайно

#define TM_FOLDER_BIT           2       //номер бита
#define TM_FOLDERORDERMASK      4       //маска порядка воспроизведения
#define TM_FOLDER_ANY           0       //любая папка
#define TM_FOLDER_ONE           4       //только внутри папки

//флаги вывода звука            
#define TM_NOSOUND              8       //без звука при воспроизведении MP3
#define TM_NOVOICE              16      //без озвучки(голос действия нажатия кнопок)

//оптимизация
#define TM_OPT_BIT              5
#define TM_OPT                  32      //оптимизация фреймов

//ИСТОЧНИК ЭФФЕКТОВ
#define TM_SRC_MASK             96      //маска источника эффектов
#define TM_NO_SRC               0
#define TM_OSCIL                64      //осциллоскоп-при ILDA автоматически задействует микрофон
#define TM_MIC                  128     //включен микрофон
//ЭФФЕКТЫ
#define TM_EFFECTMASK           7936     //маска эффектов
#define TM_BREAK                256      //приостановка по басс-детектору от микрофона
#define TM_ROTATE               512      //вращение по сч-детектору от микрофона
#define TM_SIZE                 1024     //увеличение по вч-детектору от микрофона
#define TM_FLASH                2048     //вспышка
#define TM_STROB                4096     //стробоскоп

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

//----------------------------------------------------------------------------//
#endif /* __LSCAN_TYPES_H */
/******************* (C) COPYRIGHT 2015 dem1305 *****END OF FILE****/
