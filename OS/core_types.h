/**
  Laser project by dem1305

  @2017
  
  ���� ����������� ����� ������
*/
//----------------------------------------------------------------------------//
#ifndef __CORE_TYPES_H
#define __CORE_TYPES_H
//----------------------------------------------------------------------------//
#include "core.h"
//----------------------------------------------------------------------------//
/*
������������ ���������� ������ �������������=32 �������.
*/
#define MAXIMUM_WAIT_OBJECTS        32 //32 ���� (����� ����������� �����������)
//----------------------------------------------------------------------------//
//sync core wait status
#define WAIT_TIMEOUT                ((DWORD)0x00000102L) //����� �����
#define STATUS_WAIT_0               ((DWORD)0x00000000L) //+������ ����������� ������� 
#define STATUS_ABANDONED_WAIT_0     ((DWORD)0x00000080L) //   
#define WAIT_FAILED                 ((DWORD)0xFFFFFFFF)  //�������� ��������� � �������
#define WAIT_OBJECT_0               ((STATUS_WAIT_0) + 0 )
#define WAIT_ABANDONED         ((STATUS_ABANDONED_WAIT_0 ) + 0 )
#define WAIT_ABANDONED_0       ((STATUS_ABANDONED_WAIT_0 ) + 0 )
//----------------------------------------------------------------------------//
typedef int TFUNC(void* pArgs);//�������� ������� ������
//----------------------------------------------------------------------------//
//����������� ������� = 32-������ ���������
typedef uint32_t* HANDLE;
//----------------------------------------------------------------------------//
//������ ���� (���������� � ��������� ������������ ������� � ������ ���������)
#pragma pack(1)
typedef struct        //--------coreobj------->>
{
  uint16_t   _ty;      //��� ������� 
  uint16_t   flags;    //�����
  //----sync-------
  HANDLE*  sync;       //������ ������ ������������� (������ ���������� �� �������)
  //----link-------
  HANDLE prev;         //���������� ��������
  HANDLE next;         //��������� ��������
}COREOBJ,*PCOREOBJ;    //--------coreobj-------<<
//4*32 bit

//----------------------------------------------------------------------------//
typedef struct _SECURITY_ATTRIBUTES {
    DWORD nLength;
    LPVOID lpSecurityDescriptor;
    BOOL bInheritHandle;
} SECURITY_ATTRIBUTES, *PSECURITY_ATTRIBUTES, *LPSECURITY_ATTRIBUTES;
//----------------------------------------------------------------------------//
//�������� ����������,������� �� ����������� ��� ����� � ����������
#pragma pack(1)
typedef struct
{
  uint32_t R4;
  uint32_t R5;
  uint32_t R6;
  uint32_t R7;
  uint32_t R8;
  uint32_t R9;
  uint32_t R10;
  uint32_t R11;
}_R4_R11_regs; //8*4=32 �����
#pragma pack()

#pragma pack(1)
typedef struct
{
  HANDLE   ctx;       //�������� ������
  uint32_t flags;     //R2 ����� CF_WAIT_ALL|CF_WAIT_MULTIPLE
  HANDLE*  sync;      //������ ������ ������������� (������ ���������� �� �������)
  uint32_t sync_size; //���������� �������� �������������
  uint32_t wait_ms;   //����� �������� ����������� ��������� �������� �������������
}SYNC_PAR;
#pragma pack()

//�������� ������
#pragma pack(1)
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //4*4
  //--------coreobj-------<<
  //----mode-------
  //�����,����: 
  //[0= priv=0/npriv=1]
  //[1= MSP=0/PSP=1]
  //[2= FPU/reserved]��� ��������� ������������� ���������� ���������� ��� FPU
  //[3..7=������� ����������](0-����� ������������ 31-������ ���������)
  //[8-crit section]
  //[9-suspended]
  //[10-set stack]���������� ������ �����
  uint32_t mode;     
  uint32_t id;  //������������� ������ (int,�������������)
  //6*4
  //----------------------------------------------
  uint32_t tstack_PSP;    //������� ������� ����� PSP
  uint32_t tstaRAM_PSP;   //������ ���������� ������� ��� ����� (��� �������� � ������������)
  uint32_t tstsize_PSP;   //������ �����
  //9*4
  uint32_t rsv0;//���������
  uint32_t rsv1;
  uint32_t rsv2;
  //12*4
  //----------------------------------------------
  uint32_t wait_ms; //����� ����� INFINITE=(ULONG)-1-������ ��� ���� 
  //����� �������� ����������� ��������� �������� �������������
  //12 * 4
  //----------------------------------------------
  uint32_t sync_size; //���������� �������� �������������
  //14 * 4
  //----------------------------------------------
  _R4_R11_regs;  //8 regs=8 * 4 
  //22 x uint32_t  = 88 ������ �����
}TCONTEXT,*PTCONTEXT;
#pragma pack()
typedef TCONTEXT *HTHREAD,*PTHREAD ;

//----------------------------------------------------------------------------//
/*
����������� ��������� ������ ���������� � ����� ������ ��� �������������� DEADLOCK.
��� ������������� MF_RECURSIVE ������� ���� ��������.
����������� 1 ��� ��� ��������,���� ������� ���,������� ��� ��������.
*/
#define MF_PLAIN      ((uint16_t)CF_PLAIN)       //��� �������� ���������� �������
#define MF_RECURSIVE  ((uint16_t)CF_RECURSIVE)   //��������� ������� ���������� ���������,������� ������� count
#define MF_TIMED      ((uint16_t)CF_TIMED )      //������� ������� ������� �� ������������ �������

#pragma pack(1)
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //--------coreobj-------<<
  uint32_t  timeout;    //����� �� ������ �������(�������� ��������),
  uint32_t  timecount;  //������� ����� �� ������ �������,
  uint32_t  count;      //������� ��������� �������� 
}Mutex;
#pragma pack()
typedef Mutex *HMUTEX,*PMUTEX ;
//----------------------------------------------------------------------------//
#define EF_AUTORESET    ((uint16_t)CF_AUTORESET) //������� � ����������� (�������������)
#pragma pack(1)
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //--------coreobj-------<<
}Event;
#pragma pack()
typedef Event *HEVENT,*PEVENT ;
//----------------------------------------------------------------------------//
#pragma pack(1)
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //--------coreobj-------<<
  uint32_t  maxcount;      //������������ ���������� ���������� �������� ;
  uint32_t  count;         //������������ ������� ���������� �������� ;
}Semaphore;
#pragma pack()
typedef Semaphore *HSEMAPHORE,*PSEMAPHORE ;
//----------------------------------------------------------------------------//
//��������� ������,����� ��� ���� �������.���������� 1 ��.
//���������� � ��������� ����������.
//��������� �������,����� �������������

//������ � ����������� ��� ������� ������� (�������������),�� ��������� 0
#define ST_AUTORESET      ((uint16_t)CF_AUTORESET)  //4==���������� ��� �������������

#define ST_NO_RLD_MSK     ((uint16_t)256)   //�� ������������� ��� ���������� ������ ����� (0)
#define ST_DECR_CNT_MSK   ((uint16_t)512)   //��� �������� - ������������ 
#define ST_DIS_INT_MSK    ((uint16_t)1024)  //��������� ���� � ������� ������������
#define ST_DIS_CNT_MSK    ((uint16_t)2048)  //��������� ����

#pragma pack(1)
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //--------coreobj-------<<
  uint16_t period;            //������ ������� � ��
  uint16_t counter;           //������� �������
  void (*ITFunc)(void )  ;    //������� ����������
}SystemTimer;
#pragma pack()
typedef SystemTimer *HSYSTIMER,*PSYSTIMER ;

//------system timers macros------
#define _isSysTimerDecr(_pt)              ((_pt)->flags & ST_DECR_CNT_MSK)     //������������ ������ ���������
#define _isSysTimerIncr(_pt)              (((_pt)->flags & ST_DECR_CNT_MSK)==0)//������������ ������ ���������

#define _isSysTimerCounterDisabled(_pt)   ((_pt)->flags & ST_DIS_CNT_MSK)      //���� ��������
#define _isSysTimerCounterEnabled(_pt)    (((_pt)->flags & ST_DIS_CNT_MSK)==0) //���� ��������

#define _isSysTimerReloadDisabled(_pt)    ((_pt)->flags & ST_NO_RLD_MSK)      //������������ ���������
#define _isSysTimerReloadEnabled(_pt)     (((_pt)->flags & ST_NO_RLD_MSK)==0) //������������ ���������

#define _isSysTimerITDisabled(_pt)        ((_pt)->flags & ST_DIS_INT_MSK)      //������ ����� � ������� ������������
#define _isSysTimerITEnabled(_pt)         (((_pt)->flags & ST_DIS_INT_MSK)==0) //�������� ���� � ������� ������������
//----------------------------------------------------------------------------//
//���������� ������
/*
����� ���� ���������������� ��� ������� ������� ���������� ������� ����������.
� ������ numExecutes!=0 ,����� ������ ��������,����� ����� ����� �������,����� 
� ������ ������������� ������ ����� ����������� �� ��������.
*/
#define TF_FREE_ARGS 2
#define TF_PERIODIC  4

typedef struct
{
  //--------coreobj------->>
  COREOBJ;                        //+����� ��������/����������
  //--------coreobj-------<<
  TFUNC* pTFunc;                  //������� ������ int (*pTFunc)(void* pargs) 
  void* pargs;                    //���������
  uint32_t period;                 //������ ����������/�������� ����������
  
  uint32_t nextRun;                //��������� ���������� ����������
  uint32_t numExecutes;            //������ ����������
}Task;

typedef Task *HTASK,*PTASK ;
//----------------------------------------------------------------------------//

#endif // __CORE_TYPES_H -----2017 @MW dem1305 EOF