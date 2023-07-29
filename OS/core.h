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
Операционная среда,вытесняющая модель выполнения.
Каждый поток использует свой стек PSP
Прерывания используют стек MSP

При создании потоков задаются параметры привилегий,режим запуска и приоритет.
Используются объекты синхронизации и функции приостановки потоков.
Приостановленный/ждущий поток исключается планировщиком из времени исполнения на
время ожидания.

Объекты ядра связаны в двунаправленный список.
*/
//----------------------------------------------------------------------------//

#include <stdlib.h>
#include <string.h> //mem... in DLib_Product
#include "math.h"
//===========
#include "stm32f4xx.h"
#include "Hardware.h"
//===========
#include "CCM_MM_RAM.h"  //менеджер памяти (64 КБ памяти Core-Couped-Memory+128 основной)
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
//наиболее приоритетный поток имеет уровень 31, низкоприоритетный=0
#define DEFAULT_TPRIORITY           16    //уровень приоритета потока по умолчанию [0-31]

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

// ФЛАГИ ДЛЯ CreateThread
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
//бесконечный цикл для фиксирования ошибок при отладке
void dbg_error_led(uint32_t errormask_R0);
//------------------------------------------------------------------------------
#define _aligndown_(_vptr,_al) (((uint32_t)(_vptr))&( (uint32_t)~((_al)-1)))

extern volatile PCOREOBJ _proot_;         //якорь списка объектов ядра
extern volatile PCOREOBJ _pend_;          //конец списка объектов ядра
extern volatile HTHREAD  pctcon;          //текущий контекст активного потока 
extern volatile uint32_t tswFLdis0;       //флаги переключений SW flags
extern volatile HTHREAD _pend_thread_;    //ожидающий

#define CoreObjListAnchor       _proot_
#define CoreObjListEnd          _pend_
#define CurrActiveThreadCTX     pctcon
#define CurrentActiveThreadCTX  pctcon

extern volatile uint32_t SystemTicks;
void led_loop();
extern volatile uint32_t _otot_;
#define _core_objects_total _otot_

//----------------------------CLOSE HANDLE--------------------------------------
BOOL CloseHandle(HANDLE pco);//освободить ресурсы дескриптора объекта
//-------------------------CREATE,EXIT/DELETE THREADS---------------------------
HANDLE CreateThread(TFUNC* pFunc,void* pArgs,uint32_t mode_flags,uint32_t stack_size);
#define DeleteThread(pctx) call_delete_thread((pctx),TRUE)
extern DWORD ExitThread(DWORD dwCode);
//--------------------------CRITICAL SECTION------------------------------------
#define EnterCriticalSection __asm("svc #12") //запрещает переключение контекста текущего потока
#define LeaveCriticalSection __asm("svc #13")//разрешение переключения контекста планировщиком
//--------------------THREAD SLEEP/SUSPEND/RESUME-------------------------------
/*
приостановить поток на время в миллисекундах.
timeMs==0-отдать квант времени другому потоку.
Произойдет переключение контекстов,ядро установит флаг пропуска одного переключения.
timeMs==INFINITE-поток в вечной спячке,вывести из этого режима его может другой поток.
В критической секции поток не будет приостановлен,но приостановится при выходе из секции.
Если планировщик не найдет подходящий поток для переключения,
ожидание будет игнорировано(Например,другой поток находится в ожидании
объектов синхронизации).
*/
#define Sleep(time_Ms) call_set_sleep(0,(time_Ms));
#define FreeCPU() Sleep(0)//отдать квант времени исполнения другому потоку

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
Создать мьютекс,в случае успеха вернет дескриптор.
Если не удалось создать,вернет NULL

Ограничений на захват мьютекса владельцем нет.
Если мьютекс рекурсивный-необходимо освобождать его соттветствующее количество раз.
Если bInitialOwner!=0, мьютекс будет захвачен сразу создавшим его потоком.
  флаги : 
  MF_PLAIN      ==0-нет контроля повторного захвата
  MF_RECURSIVE  ==1-повторные захваты владельцем допустимы,ведется счетчик count
  MF_TIMED      ==2-ведется счетчик времени до освобождения захвата
*/
HANDLE CreateMutex(uint16_t flagsMask,BOOL bInitialOwner,uint32_t timeout);

//Освобождает мьютекс один раз.
//Попытка освободить мьютекс не владельцем(или свободный)вернет 0. 
BOOL ReleaseMutex( _In_ HANDLE hMutex );
//----------------------------------------------------------------------------//
DWORD WaitForMultipleObjects(
  _In_       DWORD  nCount,         //количество объектов синхронизации
  _In_ const HANDLE *lpHandles,     //массив  объектов синхронизации
  _In_       BOOL   bWaitAll,       //ожидать все объекты?(TRUE=все,0=любой один)
  _In_       DWORD  dwMilliseconds  //время ожидания ( 0=выход без ожидания !)
);

DWORD WaitForSingleObject(
  _In_ HANDLE hHandle,        //объект синхронизации
  _In_ DWORD  dwMilliseconds  //время ожидания ( 0=выход без ожидания !)
);
//----------------------------------------------------------------------------//
/*
Создать событие,в случае успеха вернет дескриптор.
Если не удалось создать,вернет NULL
*/
HANDLE CreateEvent 
(
_In_ BOOL bManualReset,	// тип сброса TRUE=ручной
_In_ BOOL bInitialState	// начальное состояние TRUE=сигнальное
);
//установить событие в сигнальное состояние.В случае успеха вернет не 0.
BOOL SetEvent(HANDLE hEvent);
//установить событие в несигнальное состояние.В случае успеха вернет не 0.
BOOL ResetEvent(HANDLE hEvent);
//----------------------------------------------------------------------------//
/*
Создать семафор,в случае успеха вернет дескриптор.
Если не удалось создать,вернет NULL
*/
HANDLE CreateSemaphore(
  _In_     LONG                  lInitialCount, //инициализация счетчика доступов
  _In_     LONG                  lMaximumCount  //максимально доступов
);

BOOL ReleaseSemaphore( _In_ HANDLE hs );//освобождает один раз
//----------------------------------------------------------------------------//
/*   СИСТЕМНЫЕ ТАЙМЕРЫ,КОНТЕКСТ ПРИЛОЖЕНИЯ (ДОСТУП ДЛЯ ВСЕХ ПОТОКОВ).
     ВЫПОЛНЕНИЕ В КОНТЕКСТЕ ПРЕРЫВАНИЯ !!!
*/
//установить инкрементный таймер с перезагрузкой.Возврат указателя таймера или 0 в случае неудачи.
//параметры- период,указатель на функцию обратного вызова прерывания
SystemTimer* SetPeriodicSystemTimer(uint16_t periodMs,void (*ITFunc)(void ));
//установить декрементный таймер без перезагрузки.Возврат указателя таймера в массиве таймеров или 0 в случае неудачи.
//параметры- период,указатель на функцию обратного вызова прерывания
SystemTimer* SetOnePulseSystemTimer(uint16_t periodMs,void (*ITFunc)(void ));
//установить новый таймер.Возврат указателя таймера в массиве таймеров или 0 в случае неудачи.
//параметры- период,указатель на функцию обратного вызова прерывания,маска признаков
//признаки: перезагружаемый/нет,инкрементный/декрементный
SystemTimer* SetSystemTimer(uint16_t periodMs,void (*ITFunc)(void ),uint16_t flags);
//обнулен ли счетчик таймера
BOOL isSystemTimerCounterNull(SystemTimer* pTimer);
//повторный перезапуск таймера.Возврат значения перезагрузки или 0 в случае неудачи.
uint16_t RestartSystemTimer(SystemTimer* pTimer);
//приостановить/продолжить
BOOL PauseSystemTimer(SystemTimer* pTimer);
BOOL ResumeSystemTimer(SystemTimer* pTimer);
BOOL DeleteSystemTimer(SystemTimer** ppTimer);//удалить
void Delay(uint16_t nTimeMs);       //задержка в миллисекундах
BOOL IsSystemTimer(HANDLE hTimer) ; //является ли указатель указателем на системный таймер
//----------------------------------------------------------------------------//
//отложенная задача.Возможно создание в прерывании (атомарное подключение)
/*
может быть сконфигурирована как функция таймера потокового времени исполнения.
В случае numExecutes!=0 ,будет отсчет запусков,после этого будет удалена,иначе 
в случае периодической задачи будет повторяться до удаления.
*/
//создание [периодической] задачи [с ограниченным количеством запусков]
void CreateDefferedTask(TFUNC*_pTFunc,
                        void* _pargs,
                        uint16_t flags,
                        uint32_t period,
                        uint32_t numExecutes);

//создание простой задачи
inline void CreateTask(TFUNC*_pTFunc,void* _pargs)
{
  CreateDefferedTask(_pTFunc,_pargs,0,0,0);
}
//-------------
//создание периодической задачи
inline void CreatePeriodicTask(TFUNC*_pTFunc,
                               void* _pargs,uint32_t period)
{
  CreateDefferedTask(_pTFunc,_pargs,TF_PERIODIC,period,0);
}
//-------------
//создание периодической задачи с аргументами выделения памяти и времени исполнения
//может использоваться для расмещения в MM для взаимодействия на уровне драйверов
inline void CreatePeriodicTaskWithFlags(TFUNC*_pTFunc,void* _pargs,uint32_t period,uint16_t flags)
{
  CreateDefferedTask(_pTFunc,_pargs,TF_PERIODIC|flags,period,0);
}
//-------------
//создание задачи с ограниченным количеством запусков
inline void CreateLimitedNumExecutesTask(TFUNC*_pTFunc,
                                         void* _pargs,uint32_t numExecutes)
{
  CreateDefferedTask(_pTFunc,_pargs,TF_PERIODIC,0,numExecutes);
}
//--------------

//----------------------------------------------------------------------------//
extern volatile uint32_t SystemTicks; //счетчик миллисекунд
#define _get_timestamp SystemTicks
//----------------------------------------------------------------------------//
//Список отложенных задач (список ядра).Задачи можно добавлять в прерываниях.
extern volatile Task* _taskList_;
/*
//инструкции блокирования для синхронизации доступа потоков к списку задач
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
extern void     __out_R0_to_stack_R1_ctx(uint32_t _R0,HTHREAD ht_R1);//вывод R0 в стек потока R1
extern void     __out_R0_to_current_stack_ctx(uint32_t _R0_val);

extern void     call_save_thread_sync_parameters(SYNC_PAR* _sync_par_);//+
//R0=объект ядра R1=(uint16_t)flags R2=SET/RESET R3=поток для вывода.Если не указан-вывода не будет.
extern HANDLE    call_change_core_object_flags(PCOREOBJ pco_R0,uint16_t flags_R1,uint8_t SET_RESET_R2,HTHREAD ht_R3);//+

//освободить объект.вернет TRUE,если успешно,если указан поток для вывода в стек
extern BOOL      call_rls_object(PCOREOBJ pco_R0,HTHREAD ht_R1);//+
extern PCOREOBJ  call_add_core_obj(PCOREOBJ pobj,HTHREAD ht_R1);//+

extern int       call_get_set_thread_prio(HTHREAD pCTX_R0,uint8_t priority_R1,HTHREAD ht_R2,uint8_t bget_set_R3);//+
extern uint32_t  call_set_suspend(HTHREAD hthrd_R0, uint32_t suspendMask_R1,HTHREAD ht_R2);//+
extern void      call_set_sleep(HTHREAD hthrd, uint32_t timeMs);//приостановить поток hthrd на timeMs миллисекунд

extern HTHREAD   select_next_SW_thread();//pendsv.s
extern void      sub_switch_context_to_R0_(HTHREAD ht_R0);//pendsv.s

//SVC #31 запуск нового потока.если _R1==0-вывода в стек не будет.
extern uint32_t  call_create_new_thread(uint32_t args,HTHREAD ht_R1);//+
extern HANDLE    svc_new_thread(uint32_t args,HTHREAD ht_R1);        //+ only core accept
extern HANDLE    call_delete_thread(HANDLE pctx,HTHREAD ht_R1);      //+ удаление потока

#define _set_priveleged_mode     __asm("svc #5")
#define _set_notpriveleged_mode  __asm("svc #4")

extern BOOL   call_CloseHandle (HANDLE pco_R0,HTHREAD ht_R1);//если _R1==0-вывода в стек не будет.
extern HANDLE call_find_handle (HANDLE ho_R0,HTHREAD ht_R1); //#23 
 //ONLY CORE ACCESS
extern int      svc_get_thread_id(HTHREAD ht_R0,HTHREAD ht_outstack_R1);//#10 only core accept return -1=error
extern HANDLE   svc_get_thread_handle(int id,HTHREAD ht_outstack_R1);  //#11 only core accept return -1=error
//#3 R0=pctx R1=prio передача через стек R2=поток для вывода в стек R3=0==get/1==set
extern int svc_get_set_thread_prio(HTHREAD pCTX_R0,uint8_t priority_R1,HTHREAD ht_R2,uint8_t bget_set_R3);//+

extern int     call_GetThreadId(HTHREAD ht_R0,HTHREAD ht_outstack_R1);
extern HANDLE   call_GetThreadHandle(int id,HTHREAD ht_outstack_R1);//return -1=error
//----------------------------------------------------------------------------//
//бесконечный цикл для фиксирования ошибок при отладке
void dbg_error_led(uint32_t errormask_R0);//core.c
//----------------------------------------------------------------------------//
HTHREAD _check_for_sync(HTHREAD ht);//core.c проверка на синхронизацию
bool core_find_CoreObj(PCOREOBJ pobj);//core.c найти объект ядра
//добавить объект к списку.Если поток не указан, вывода в стек не будет.
PCOREOBJ core_add_CoreObj(PCOREOBJ pobj,HANDLE ht_R1);//core.c

BOOL _rls_object(HANDLE pco_R0,HTHREAD ht_R1);//sync.c

bool    core_find_CoreObj(PCOREOBJ pobj);         //найти объект ядра
PCOREOBJ core_cut_CoreObj_no_find(PCOREOBJ pobj); //вырезать объект ядра(без поиска)
PCOREOBJ core_cut_CoreObj(PCOREOBJ pobj);         //вырезать объект ядра

//----------------------------------------------------------------------------//
extern void it_mDelay (uint32_t msec);
//----------------------------------------------------------------------------//

extern BOOL call_IsSystemTimerCounterNull(SystemTimer* pTimer);
extern uint16_t call_RestartSystemTimer(SystemTimer* pTimer);
extern BOOL call_PauseSystemTimer(SystemTimer* pTimer);
extern BOOL call_ResumeSystemTimer(SystemTimer* pTimer);
extern BOOL call_DeleteSystemTimer(SystemTimer** ppTimer);//удалить
extern BOOL call_IsSystemTimer(HANDLE hTimer) ; //является ли указатель указателем на системный таймер
//----------------------------------------------------------------------------//

//принадлежит ли прерывание к группе прерываний ядра
#define _is_not_core_IT(ITnum) ((ITnum) && (ITnum)!=11 && (ITnum)!=14 && (ITnum)!=(TSV_TIMER_IRQ+16))
//приоритет прерывания более приоритета группы прерываний ядра ?
#define _is_prio_Lvl_more_Core(ITnum) (GetITPrioLevelAndStoreEncoded((IRQn_Type)((ITnum)-16) \
                                            ,CORE_NVIC_PriorityGroup,0)<CORE_IT_PRIO_GROUP)
/*
проверка прерываний.Если есть, то проверяются условия приоритета.
если приоритет текущего прерывания выше приоритета прерываний ядра,то функция 
блокирует дальнейшее выполнение во избежание разрушения структур ядра.
При удовлитворительных условиях функция вернет 0 при отсутствии прерывания либо номер
прерывания,прохождение функции не блокируется.
*/
uint32_t check_if_IT_less_or_equal();//core.c
//----------------------------------------------------------------------------//
/*
инструкции блокирования для выявления доступа более приоритетного прерывания
(вызовет зависание,т.к. доступ имеет только ядро)
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
//последняя операция ядра. Применяется для отладки.
extern volatile unsigned char _dbg_op[CORE_DBG_LEN];
extern volatile unsigned char _dbi;

#define _save_op(_numop_) {if(++_dbi>= CORE_DBG_LEN )_dbi=0;_dbg_op[ _dbi ]=(_numop_);}
#else
#define _save_op(_numop_) 
#endif

//----------------------------------------------------------------------------//
#endif//__CORE_H