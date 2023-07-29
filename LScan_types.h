/**
  Laser project by dem1305

  @2014
*/


/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __LSCAN_TYPES_H
#define __LSCAN_TYPES_H

//----------------------------------------------------------------------------//
#include "integer.h" //����
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
#define INFINITE  ((uint32_t)0xFFFFFFFF) //((ULONG)-1) 4 �����
//----------------------------------------------------------------------------//

//---------FR_EOF--------
#define FR_EOF  FR_NOT_ENABLED //=(12) The volume has no work area */
//----------------------------------------------------------------------------//
//������ �� ��������� _fr �� ��������� _to � ��������� ������ �������� _buf_size
#define DISTRING(_fr,_to,_buf_size) ((_fr)<=(_to)?(int)((_to)-(_fr)):((int)(_buf_size)-(int)(_fr)+(int)(_to)))

#define LOBYTE(w)           ((BYTE)(((WORD)(w)) & 0xff))
#define HIBYTE(w)           ((BYTE)((((WORD)(w)) >> 8) & 0xff))
//----------------------------------------------------------------------------//
//command
typedef enum
{
  IC_NO=0,
  IC_DPSS_ON=1,                 //���������� �������� ������ DPSS � �� ������������ ��������� ��������
  IC_GET_STREAM_PAK_SIZE=128,		//�������� ������ ������ ���������� ����� � 8-������� ��������
  IC_GET_STREAM_BUF_SIZE=129,		//�������� ������ ���������� ������ � ������� ���������� �����
  IC_GET_BUF_FREE_PACKS=130,		//�������� ��������� ����� � ������� ���������� ����� � ��������� ������
  IC_SET_USB_DISABLE=131,				//���������� ��������� USB
  IC_GET_BUF_FREE_256=132,			//�������� ��������� ����� � 256-������� ������
  IC_GET_BUF_FREE_8=133,				//�������� ��������� ����� � ������/8
  IC_GET_LVER=134							  //���� , ������� ������ LScan (LSCAN_VERSION_NUM)
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
����������� � COORDSTR ��������� ������������ ������������ �����. ������ 2
*/
#pragma pack(1)
typedef struct 
{
  uint8_t cmd;    //�������
  uint8_t discrMs;//������������� , ����� � ������������
  uint8_t lx ;    //������� ����� ���������
  uint8_t ly;
  uint8_t hxhy;   //������� ��������� ���������
  uint8_t r;	    //1 ���� ����, 0-255
  uint8_t g;	    //1 ���� ����, 0-255
  uint8_t b;	    //1 ���� ����, 0-255
}ICP;
#pragma pack()
//8 ���� 
//----------------------------------------------------------------------------//

//������������ 1 ����!!!!!!
#pragma pack(1)
typedef struct {
  UCHAR		headerSym[4];// �I�, �L�, �D�, �A�  The ASCII letters ILDA, identifying an ILDA format header. 
  UCHAR		zBytes[3];   //��� ����� ����� . �� ������������, �� ������ ���� �������.
  UCHAR		format;      // ������-��� 2D(3D) ��������; 0 ��� 3-D ��� ��������� 1 ��� 2-D. � 3-D ������ ������ ����� �� ����� � � 2-D ������ ��� ����� �� �����.
  UCHAR		frameName[8];//������ �������� ASCII � ������ ����� �����.
  UCHAR		company[8];  //������ �������� ASCII � ������ ��������, ������� ������� ����(�����). 
  USHORT 	pointsTotal; //���������� ����� � ������� �����(0-65535), ���� 0,����� ��� ����������� �� ����� �����(���������) � ������ ������ �� �����.
  USHORT 	frameNum;    //���� ����� - ����� ������ ����� ��� ������������������ ��������, ��� ������������ ����� �����. ������� ���������� � ������ 0. �������� ������ 0-65535.
  USHORT 	framesTotal; //����� ������� � ���� ������ ��� ������������������.�������� 1-65535. 
  UCHAR		scannerNum;  //����� ������� ��� ���������.�������� 0-255. ��� LScan ����� 0.
  UCHAR 	future;      //�������������� ��� �������� �������������. ������ ���� ���������� � 0.
} ILDA_COORD_HDR;//32 bytes

typedef struct {
  UCHAR		headerSym[4];// �I�, �L�, �D�, �A�  The ASCII letters ILDA, identifying an ILDA format header. 
  UCHAR		zBytes[3];   //��� ����� ����� . �� ������������, �� ������ ���� �������.
  UCHAR		format;      // ������-���  2 ��� ������� ������
  UCHAR		paletteName[8];//������ �������� ASCII � ������ ���� �������.
  UCHAR		company[8];  //������ �������� ASCII � ������ ��������, ������� ������� ����(�����). 
  USHORT 	colorsTotal; //����� ������ � ������� (�� 2)
  USHORT 	paletteNum;    //����� �������. ������� ���������� � 0. ��������  0-65535.
  USHORT 	future;      //�������������� ��� �������� �������������. ������ ���� ���������� � 0.
  UCHAR		scannerNum;  //����� ������� ��� ���������.�������� 0-255. ��� LScan ����� 0.
  UCHAR 	future1;      //�������������� ��� �������� �������������. ������ ���� ���������� � 0.
} ILDA_COLOR_HDR;//32 bytes

//ILDA
typedef struct{
  short x;
  short y;
  short z;//�� ������������ � 2D
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
  int x; //��������� �������
  int y; //��������� �������
  int w; //������
  int h; //������
} RECT;

typedef struct //vector movement structure for optimisation
{
  float v0x;
  float v0y;//����������� �������� �� ����
}OPT_V;
//----------------------------------------------------------------------------//
#define LSCAN_VERSION_NUM   2
//INTERNAL STATUS RESULT
typedef   uint8_t IS_RES;

#define   IS_OK     ((uint8_t)0x0)     //0 ==�������
#define   IS_ERR    ((uint8_t)0x1)     //1 ==������
#define   IS_EFS    ((uint8_t)0x2)     //2 ==������ �������� �������
#define   IS_PLAY   ((uint8_t)0x4)     //4 ==������������
#define   IS_EOF    ((uint8_t)0x8)     //8 ==����� �����
#define   IS_EODIR  ((uint8_t)0x10)    //16 ==����� ����������
#define   IS_BUSY   ((uint8_t)0x20)    //32==�����
//----------------------------------------------------------------------------//
//���������� �� ������� � ����������� �� n ������������������ ,n>1-�������� � �2
#define _crdi(c1,c2,n)     (((c1)+(n)*(c2))/(1+(n)))  
//logxY = log Y / log X -�������� �� ��������� x ����� Y
#define logx(base,x) (log10(x)/log10(base))
//������ �� ��������� _fr �� ��������� _to � ��������� ������ �������� _buf_size
#define bytesCircularBufferFromTo(_fr,_to,_buf_size) ((_fr)<=(_to)?(int)((_to)-(_fr)):((int)(_buf_size)-((int)(_fr)-(int)(_to))))
//----------------------------------------------------------------------------//
#define ZeroMemory(mem,len) memset((mem),0,(len))
#define Clear(obj) ZeroMemory((obj),sizeof(obj))
#define _MHZ *1000000
//----------------------------------------------------------------------------//
//servo parameters
typedef struct{
  uint16_t min;   //����� ������� ���������, CLK
  uint16_t max;   //������ ������� ���������, CLK
  uint16_t angle; //�������� �������� � ��������
} SERVO_PAR;

//PWM parameters
typedef struct{
  uint32_t clk_sec;//������������, ������ � �������
  uint16_t prescaler;//������������
  uint16_t period;//����� ���
} PWM_PAR;


//��� ��������������� ������ � ����� ��������� �������� ��� ���������������.
typedef uint16_t TYPEMODEMASK;

//����� ����� �������������� � ����� 
#define TM_FILETYPE_BIT         0       //����� ����
#define TM_FILETYPEMASK         1       //����� ������ ��������������� ILDA/MP3
#define TM_ILDA                 0       //������� ����� ��������������� ILDA
#define TM_MP3                  1       //����� MP3
//��� �������� ����� � �����
#define TM_FILEORDER_BIT        1       //����� ����
#define TM_FILEORDERMASK        2       //����� ������� ���������������
#define TM_FILE_ORDER           0       //�� �������
#define TM_FILE_RANDOM          2       //��������

#define TM_FOLDER_BIT           2       //����� ����
#define TM_FOLDERORDERMASK      4       //����� ������� ���������������
#define TM_FOLDER_ANY           0       //����� �����
#define TM_FOLDER_ONE           4       //������ ������ �����

//����� ������ �����            
#define TM_NOSOUND              8       //��� ����� ��� ��������������� MP3
#define TM_NOVOICE              16      //��� �������(����� �������� ������� ������)

//�����������
#define TM_OPT_BIT              5
#define TM_OPT                  32      //����������� �������

//�������� ��������
#define TM_SRC_MASK             96      //����� ��������� ��������
#define TM_NO_SRC               0
#define TM_OSCIL                64      //�����������-��� ILDA ������������� ����������� ��������
#define TM_MIC                  128     //������� ��������
//�������
#define TM_EFFECTMASK           7936     //����� ��������
#define TM_BREAK                256      //������������ �� ����-��������� �� ���������
#define TM_ROTATE               512      //�������� �� ��-��������� �� ���������
#define TM_SIZE                 1024     //���������� �� ��-��������� �� ���������
#define TM_FLASH                2048     //�������
#define TM_STROB                4096     //����������

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
