/**
  Laser project by dem1305

  @2017-2018
  
  OS core
*/
//----------------------------------------------------------------------------//
#ifndef __CORE_H
#define __CORE_H
//----------------------------------------------------------------------------//
/*
������������ �����,����������� ������ ����������.
������ ����� ���������� ���� ���� PSP
���������� ���������� ���� MSP

��� �������� ������� �������� ��������� ����������,����� ������� � ���������.
������������ ������� ������������� � ������� ������������ �������.
����������������/������ ����� ����������� ������������� �� ������� ���������� ��
����� ��������.

������� ���� ������� � ��������������� ������.
*/
//----------------------------------------------------------------------------//

#include <stdlib.h>
#include <string.h> //mem... in DLib_Product
#include "math.h"
//===========
#include "stm32f4xx.h"
#include "Hardware.h"
//===========
#include "CCM_MM_RAM.h"  //�������� ������ (64 �� ������ Core-Couped-Memory+128 ��������)
//===========
#include "core_def.h"
#include "core_types.h"
#include "atomic.h"
#include "MPU.h"
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
//RAM 112 [192] KB = 2000 0000 - 2001 BFFF [2001 FFFF]
//CCM RAM 64 KB = 1000 0000 - 1000 FFFF
//----------------------------------------------------------------------------//
//�������� ������������ ����� ����� ������� 31, �����������������=0
#define DEFAULT_TPRIORITY           16    //������� ���������� ������ �� ��������� [0-31]

#define THREAD_BASE_PRIORITY_LOWRT  0     // value that gets a thread to LowRealtime-1
#define THREAD_BASE_PRIORITY_MAX    31    // maximum thread base priority boost
#define THREAD_BASE_PRIORITY_MIN    0     // minimum thread base priority boost

#define THREAD_PRIORITY_LOWEST          THREAD_BASE_PRIORITY_MIN
#define THREAD_PRIORITY_BELOW_NORMAL    (THREAD_PRIORITY_LOWEST+1)
#define THREAD_PRIORITY_NORMAL          DEFAULT_TPRIORITY
#define THREAD_PRIORITY_HIGHEST         THREAD_BASE_PRIORITY_MAX
#define THREAD_PRIORITY_ABOVE_NORMAL    (THREAD_PRIORITY_HIGHEST-1)
#define THREAD_PRIORITY_ERROR_RETURN    (MAXLONG)

#define THREAD_PRIORITY_TIME_CRITICAL   THREAD_BASE_PRIORITY_LOWRT

// ����� ��� CreateThread
#define TM_PRIORITY_LOWEST          (((uint32_t)THREAD_PRIORITY_LOWEST)<<TPRIO_BIT_Pos)
#define TM_PRIORITY_BELOW_NORMAL    (((uint32_t)THREAD_PRIORITY_BELOW_NORMAL)<<TPRIO_BIT_Pos)
#define TM_PRIORITY_NORMAL          (((uint32_t)THREAD_PRIORITY_NORMAL)<<TPRIO_BIT_Pos)
#define TM_PRIORITY_HIGHEST         (((uint32_t)THREAD_PRIORITY_HIGHEST)<<TPRIO_BIT_Pos)
#define TM_PRIORITY_ABOVE_NORMAL    (((uint32_t)THREAD_PRIORITY_ABOVE_NORMAL)<<TPRIO_BIT_Pos)
#define TM_PRIORITY_ERROR_RETURN    (((uint32_t)THREAD_PRIORITY_ERROR_RETURN)<<TPRIO_BIT_Pos)
#define TM_PRIORITY_TIME_CRITICAL   (((uint32_t)THREAD_BASE_PRIORITY_LOWRT)<<TPRIO_BIT_Pos)
//----------------------------------------------------------------------------//
#define TPRIO_MSK           TPRIO_Msk //11111000
#define _tprio(_tmode)      (((_tmode)&TPRIO_MSK)>>TPRIO_BIT_Pos) //get thread priority 
//set thread priority
#define _TPRIORITY(_lvl_0_to_31) (((_lvl_0_to_31)<<TPRIO_BIT_Pos)&TPRIO_MSK)  
//------------------------------------------------------------------------------
//����������� ���� ��� ������������ ������ ��� �������
void dbg_error_led(uint32_t errormask_R0);
//------------------------------------------------------------------------------
#define _aligndown_(_vptr,_al) (((uint32_t)(_vptr))&( (uint32_t)~((_al)-1)))

extern volatile PCOREOBJ _proot_;         //����� ������ �������� ����
extern volatile PCOREOBJ _pend_;          //����� ������ �������� ����
extern volatile HTHREAD  pctcon;          //������� �������� ��������� ������ 
extern volatile uint32_t tswFLdis0;       //����� ������������ SW flags
extern volatile HTHREAD _pend_thread_;    //���������

#define CoreObjListAnchor       _proot_
#define CoreObjListEnd          _pend_
#define CurrActiveThreadCTX     pctcon
#define CurrentActiveThreadCTX  pctcon

extern volatile uint32_t SystemTicks;
void led_loop();
extern volatile uint32_t _otot_;
#define _core_objects_total _otot_

//----------------------------CLOSE HANDLE--------------------------------------
BOOL CloseHandle(HANDLE pco);//���������� ������� ����������� �������
//-------------------------CREATE,EXIT/DELETE THREADS---------------------------
HANDLE CreateThread(TFUNC* pFunc,void* pArgs,uint32_t mode_flags,uint32_t stack_size);
#define DeleteThread(pctx) call_delete_thread((pctx),TRUE)
extern DWORD ExitThread(DWORD dwCode);
//--------------------------CRITICAL SECTION------------------------------------
#define EnterCriticalSection __asm("svc #12") //��������� ������������ ��������� �������� ������
#define LeaveCriticalSection __asm("svc #13")//���������� ������������ ��������� �������������
//--------------------THREAD SLEEP/SUSPEND/RESUME-------------------------------
/*
������������� ����� �� ����� � �������������.
timeMs==0-������ ����� ������� ������� ������.
���������� ������������ ����������,���� ��������� ���� �������� ������ ������������.
timeMs==INFINITE-����� � ������ ������,������� �� ����� ������ ��� ����� ������ �����.
� ����������� ������ ����� �� ����� �������������,�� �������������� ��� ������ �� ������.
���� ����������� �� ������ ���������� ����� ��� ������������,
�������� ����� ������������(��������,������ ����� ��������� � ��������
�������� �������������).
*/
#define Sleep(time_Ms) call_set_sleep(0,(time_Ms));
#define FreeCPU() Sleep(0)//������ ����� ������� ���������� ������� ������

//set suspend  mask
extern int ResumeThread(HTHREAD hTrd); //return -1=error
extern int SuspendThread(HTHREAD hTrd);//return -1=error

//---------------------------SYS TIMESTAMP--------------------------------------
inline uint32_t GetSysTick(){return SystemTicks;}//system ticks 
//--------------------------THREAD ATTRIBUTES-----------------------------------
//---get---ID / HADLE ------
int GetThreadId(HTHREAD hTrd);//return -1=error
#define GetCurrentThreadId() GetThreadId(0) //return -1=error
HANDLE GetThreadHandle(int id);//return -1=error
#define GetCurrentThreadHandle() GetThreadHandle(0) //return -1=error
//set/get prio.level
int SetThreadPriorityLevel(HTHREAD pCTX,uint8_t priority);//set priority
int GetThreadPriorityLevel(HTHREAD pCTX);

//----------------------------------------------------------------------------//
/*
������� �������,� ������ ������ ������ ����������.
���� �� ������� �������,������ NULL

����������� �� ������ �������� ���������� ���.
���� ������� �����������-���������� ����������� ��� ��������������� ���������� ���.
���� bInitialOwner!=0, ������� ����� �������� ����� ��������� ��� �������.
  ����� : 
  MF_PLAIN      ==0-��� �������� ���������� �������
  MF_RECURSIVE  ==1-��������� ������� ���������� ���������,������� ������� count
  MF_TIMED      ==2-������� ������� ������� �� ������������ �������
*/
HANDLE CreateMutex(uint16_t flagsMask,BOOL bInitialOwner,uint32_t timeout);

//����������� ������� ���� ���.
//������� ���������� ������� �� ����������(��� ���������)������ 0. 
BOOL ReleaseMutex( _In_ HANDLE hMutex );
//----------------------------------------------------------------------------//
DWORD WaitForMultipleObjects(
  _In_       DWORD  nCount,         //���������� �������� �������������
  _In_ const HANDLE *lpHandles,     //������  �������� �������������
  _In_       BOOL   bWaitAll,       //������� ��� �������?(TRUE=���,0=����� ����)
  _In_       DWORD  dwMilliseconds  //����� �������� ( 0=����� ��� �������� !)
);

DWORD WaitForSingleObject(
  _In_ HANDLE hHandle,        //������ �������������
  _In_ DWORD  dwMilliseconds  //����� �������� ( 0=����� ��� �������� !)
);
//----------------------------------------------------------------------------//
/*
������� �������,� ������ ������ ������ ����������.
���� �� ������� �������,������ NULL
*/
HANDLE CreateEvent 
(
_In_ BOOL bManualReset,	// ��� ������ TRUE=������
_In_ BOOL bInitialState	// ��������� ��������� TRUE=����������
);
//���������� ������� � ���������� ���������.� ������ ������ ������ �� 0.
BOOL SetEvent(HANDLE hEvent);
//���������� ������� � ������������ ���������.� ������ ������ ������ �� 0.
BOOL ResetEvent(HANDLE hEvent);
//----------------------------------------------------------------------------//
/*
������� �������,� ������ ������ ������ ����������.
���� �� ������� �������,������ NULL
*/
HANDLE CreateSemaphore(
  _In_     LONG                  lInitialCount, //������������� �������� ��������
  _In_     LONG                  lMaximumCount  //����������� ��������
);

BOOL ReleaseSemaphore( _In_ HANDLE hs );//����������� ���� ���
//----------------------------------------------------------------------------//
/*   ��������� �������,�������� ���������� (������ ��� ���� �������).
     ���������� � ��������� ���������� !!!
*/
//���������� ������������ ������ � �������������.������� ��������� ������� ��� 0 � ������ �������.
//���������- ������,��������� �� ������� ��������� ������ ����������
SystemTimer* SetPeriodicSystemTimer(uint16_t periodMs,void (*ITFunc)(void ));
//���������� ������������ ������ ��� ������������.������� ��������� ������� � ������� �������� ��� 0 � ������ �������.
//���������- ������,��������� �� ������� ��������� ������ ����������
SystemTimer* SetOnePulseSystemTimer(uint16_t periodMs,void (*ITFunc)(void ));
//���������� ����� ������.������� ��������� ������� � ������� �������� ��� 0 � ������ �������.
//���������- ������,��������� �� ������� ��������� ������ ����������,����� ���������
//��������: ���������������/���,������������/������������
SystemTimer* SetSystemTimer(uint16_t periodMs,void (*ITFunc)(void ),uint16_t flags);
//������� �� ������� �������
BOOL isSystemTimerCounterNull(SystemTimer* pTimer);
//��������� ���������� �������.������� �������� ������������ ��� 0 � ������ �������.
uint16_t RestartSystemTimer(SystemTimer* pTimer);
//�������������/����������
BOOL PauseSystemTimer(SystemTimer* pTimer);
BOOL ResumeSystemTimer(SystemTimer* pTimer);
BOOL DeleteSystemTimer(SystemTimer** ppTimer);//�������
void Delay(uint16_t nTimeMs);       //�������� � �������������
BOOL IsSystemTimer(HANDLE hTimer) ; //�������� �� ��������� ���������� �� ��������� ������
//----------------------------------------------------------------------------//
//���������� ������.�������� �������� � ���������� (��������� �����������)
/*
����� ���� ���������������� ��� ������� ������� ���������� ������� ����������.
� ������ numExecutes!=0 ,����� ������ ��������,����� ����� ����� �������,����� 
� ������ ������������� ������ ����� ����������� �� ��������.
*/
//�������� [�������������] ������ [� ������������ ����������� ��������]
void CreateDefferedTask(TFUNC*_pTFunc,
                        void* _pargs,
                        uint16_t flags,
                        uint32_t period,
                        uint32_t numExecutes);

//�������� ������� ������
inline void CreateTask(TFUNC*_pTFunc,void* _pargs)
{
  CreateDefferedTask(_pTFunc,_pargs,0,0,0);
}
//-------------
//�������� ������������� ������
inline void CreatePeriodicTask(TFUNC*_pTFunc,
                               void* _pargs,uint32_t period)
{
  CreateDefferedTask(_pTFunc,_pargs,TF_PERIODIC,period,0);
}
//-------------
//�������� ������������� ������ � ����������� ��������� ������ � ������� ����������
//����� �������������� ��� ���������� � MM ��� �������������� �� ������ ���������
inline void CreatePeriodicTaskWithFlags(TFUNC*_pTFunc,void* _pargs,uint32_t period,uint16_t flags)
{
  CreateDefferedTask(_pTFunc,_pargs,TF_PERIODIC|flags,period,0);
}
//-------------
//�������� ������ � ������������ ����������� ��������
inline void CreateLimitedNumExecutesTask(TFUNC*_pTFunc,
                                         void* _pargs,uint32_t numExecutes)
{
  CreateDefferedTask(_pTFunc,_pargs,TF_PERIODIC,0,numExecutes);
}
//--------------

//----------------------------------------------------------------------------//
extern volatile uint32_t SystemTicks; //������� �����������
#define _get_timestamp SystemTicks
//----------------------------------------------------------------------------//
//������ ���������� ����� (������ ����).������ ����� ��������� � �����������.
extern volatile Task* _taskList_;
/*
//���������� ������������ ��� ������������� ������� ������� � ������ �����
extern volatile unsigned char _taskList_flag_lock;
#define _tasklistlock   ExclusiveLockUnlockBool((unsigned char*)&_taskList_flag_lock,true)
#define _tasklistunlock ExclusiveLockUnlockBool((unsigned char*)&_taskList_flag_lock,false)
*/
//----------------------------------------------------------------------------//

#define _core_disable_sw      __asm("svc #14")
#define _core_enable_sw       __asm("svc #15")
#define _core_sw_thread       __asm("svc #2")

//----------------------------------------------------------------------------//
//EXTERN ASM FUNCS
extern uint32_t  __in_R0_current_stack_ctx();
extern void     __out_R0_to_stack_R1_ctx(uint32_t _R0,HTHREAD ht_R1);//����� R0 � ���� ������ R1
extern void     __out_R0_to_current_stack_ctx(uint32_t _R0_val);

extern void     call_save_thread_sync_parameters(SYNC_PAR* _sync_par_);//+
//R0=������ ���� R1=(uint16_t)flags R2=SET/RESET R3=����� ��� ������.���� �� ������-������ �� �����.
extern HANDLE    call_change_core_object_flags(PCOREOBJ pco_R0,uint16_t flags_R1,uint8_t SET_RESET_R2,HTHREAD ht_R3);//+

//���������� ������.������ TRUE,���� �������,���� ������ ����� ��� ������ � ����
extern BOOL      call_rls_object(PCOREOBJ pco_R0,HTHREAD ht_R1);//+
extern PCOREOBJ  call_add_core_obj(PCOREOBJ pobj,HTHREAD ht_R1);//+

extern int       call_get_set_thread_prio(HTHREAD pCTX_R0,uint8_t priority_R1,HTHREAD ht_R2,uint8_t bget_set_R3);//+
extern uint32_t  call_set_suspend(HTHREAD hthrd_R0, uint32_t suspendMask_R1,HTHREAD ht_R2);//+
extern void      call_set_sleep(HTHREAD hthrd, uint32_t timeMs);//������������� ����� hthrd �� timeMs �����������

extern HTHREAD   select_next_SW_thread();//pendsv.s
extern void      sub_switch_context_to_R0_(HTHREAD ht_R0);//pendsv.s

//SVC #31 ������ ������ ������.���� _R1==0-������ � ���� �� �����.
extern uint32_t  call_create_new_thread(uint32_t args,HTHREAD ht_R1);//+
extern HANDLE    svc_new_thread(uint32_t args,HTHREAD ht_R1);        //+ only core accept
extern HANDLE    call_delete_thread(HANDLE pctx,HTHREAD ht_R1);      //+ �������� ������

#define _set_priveleged_mode     __asm("svc #5")
#define _set_notpriveleged_mode  __asm("svc #4")

extern BOOL   call_CloseHandle (HANDLE pco_R0,HTHREAD ht_R1);//���� _R1==0-������ � ���� �� �����.
extern HANDLE call_find_handle (HANDLE ho_R0,HTHREAD ht_R1); //#23 
 //ONLY CORE ACCESS
extern int      svc_get_thread_id(HTHREAD ht_R0,HTHREAD ht_outstack_R1);//#10 only core accept return -1=error
extern HANDLE   svc_get_thread_handle(int id,HTHREAD ht_outstack_R1);  //#11 only core accept return -1=error
//#3 R0=pctx R1=prio �������� ����� ���� R2=����� ��� ������ � ���� R3=0==get/1==set
extern int svc_get_set_thread_prio(HTHREAD pCTX_R0,uint8_t priority_R1,HTHREAD ht_R2,uint8_t bget_set_R3);//+

extern int     call_GetThreadId(HTHREAD ht_R0,HTHREAD ht_outstack_R1);
extern HANDLE   call_GetThreadHandle(int id,HTHREAD ht_outstack_R1);//return -1=error
//----------------------------------------------------------------------------//
//����������� ���� ��� ������������ ������ ��� �������
void dbg_error_led(uint32_t errormask_R0);//core.c
//----------------------------------------------------------------------------//
HTHREAD _check_for_sync(HTHREAD ht);//core.c �������� �� �������������
bool core_find_CoreObj(PCOREOBJ pobj);//core.c ����� ������ ����
//�������� ������ � ������.���� ����� �� ������, ������ � ���� �� �����.
PCOREOBJ core_add_CoreObj(PCOREOBJ pobj,HANDLE ht_R1);//core.c

BOOL _rls_object(HANDLE pco_R0,HTHREAD ht_R1);//sync.c

bool    core_find_CoreObj(PCOREOBJ pobj);         //����� ������ ����
PCOREOBJ core_cut_CoreObj_no_find(PCOREOBJ pobj); //�������� ������ ����(��� ������)
PCOREOBJ core_cut_CoreObj(PCOREOBJ pobj);         //�������� ������ ����

//----------------------------------------------------------------------------//
extern void it_mDelay (uint32_t msec);
//----------------------------------------------------------------------------//

extern BOOL call_IsSystemTimerCounterNull(SystemTimer* pTimer);
extern uint16_t call_RestartSystemTimer(SystemTimer* pTimer);
extern BOOL call_PauseSystemTimer(SystemTimer* pTimer);
extern BOOL call_ResumeSystemTimer(SystemTimer* pTimer);
extern BOOL call_DeleteSystemTimer(SystemTimer** ppTimer);//�������
extern BOOL call_IsSystemTimer(HANDLE hTimer) ; //�������� �� ��������� ���������� �� ��������� ������
//----------------------------------------------------------------------------//

//����������� �� ���������� � ������ ���������� ����
#define _is_not_core_IT(ITnum) ((ITnum) && (ITnum)!=11 && (ITnum)!=14 && (ITnum)!=(TSV_TIMER_IRQ+16))
//��������� ���������� ����� ���������� ������ ���������� ���� ?
#define _is_prio_Lvl_more_Core(ITnum) (GetITPrioLevelAndStoreEncoded((IRQn_Type)((ITnum)-16) \
                                            ,CORE_NVIC_PriorityGroup,0)<CORE_IT_PRIO_GROUP)
/*
�������� ����������.���� ����, �� ����������� ������� ����������.
���� ��������� �������� ���������� ���� ���������� ���������� ����,�� ������� 
��������� ���������� ���������� �� ��������� ���������� �������� ����.
��� ������������������ �������� ������� ������ 0 ��� ���������� ���������� ���� �����
����������,����������� ������� �� �����������.
*/
uint32_t check_if_IT_less_or_equal();//core.c
//----------------------------------------------------------------------------//
/*
���������� ������������ ��� ��������� ������� ����� ������������� ����������
(������� ���������,�.�. ������ ����� ������ ����)
*/
extern volatile unsigned char _core_flag_lock;
#define _corelock   _core_do_lock_unlock(true)
#define _coreunlock _core_do_lock_unlock(false)

static inline void _core_do_lock_unlock(bool _b_do_lock)
{
  uint32_t it=(__get_IPSR() & 0xFF);
    
  if(_is_not_core_IT(it))while(1);//IT not of core (!SVcall && !PendSV && !SysCoreTimer)!   
  
  ExclusiveLockUnlockBool((unsigned char*)&_core_flag_lock,_b_do_lock);
}

#if CORE_DBG_OPERATION==1
//��������� �������� ����. ����������� ��� �������.
extern volatile unsigned char _dbg_op[CORE_DBG_LEN];
extern volatile unsigned char _dbi;

#define _save_op(_numop_) {if(++_dbi>= CORE_DBG_LEN )_dbi=0;_dbg_op[ _dbi ]=(_numop_);}
#else
#define _save_op(_numop_) 
#endif

//----------------------------------------------------------------------------//
#endif//__CORE_H