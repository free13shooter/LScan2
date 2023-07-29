/**
  Laser project by dem1305

  @2017
  
  файл определений типов данных
*/
//----------------------------------------------------------------------------//
#ifndef __CORE_TYPES_H
#define __CORE_TYPES_H
//----------------------------------------------------------------------------//
#include "core.h"
//----------------------------------------------------------------------------//
/*
максимальное количество списка синхронизации=32 объекта.
*/
#define MAXIMUM_WAIT_OBJECTS        32 //32 бита (маска оптимальной разрядности)
//----------------------------------------------------------------------------//
//sync core wait status
#define WAIT_TIMEOUT                ((DWORD)0x00000102L) //вышло время
#define STATUS_WAIT_0               ((DWORD)0x00000000L) //+индекс сигнального объекта 
#define STATUS_ABANDONED_WAIT_0     ((DWORD)0x00000080L) //   
#define WAIT_FAILED                 ((DWORD)0xFFFFFFFF)  //ожидание закончено с ошибкой
#define WAIT_OBJECT_0               ((STATUS_WAIT_0) + 0 )
#define WAIT_ABANDONED         ((STATUS_ABANDONED_WAIT_0 ) + 0 )
#define WAIT_ABANDONED_0       ((STATUS_ABANDONED_WAIT_0 ) + 0 )
//----------------------------------------------------------------------------//
typedef int TFUNC(void* pArgs);//прототип функции потока
//----------------------------------------------------------------------------//
//манипулятор объекта = 32-битный указатель
typedef uint32_t* HANDLE;
//----------------------------------------------------------------------------//
//объект ядра (включается в структуру производного объекта в начале структуры)
#pragma pack(1)
typedef struct        //--------coreobj------->>
{
  uint16_t   _ty;      //тип объекта 
  uint16_t   flags;    //флаги
  //----sync-------
  HANDLE*  sync;       //массив списка синхронизации (массив указателей на объекты)
  //----link-------
  HANDLE prev;         //предыдущий контекст
  HANDLE next;         //следующий контекст
}COREOBJ,*PCOREOBJ;    //--------coreobj-------<<
//4*32 bit

//----------------------------------------------------------------------------//
typedef struct _SECURITY_ATTRIBUTES {
    DWORD nLength;
    LPVOID lpSecurityDescriptor;
    BOOL bInheritHandle;
} SECURITY_ATTRIBUTES, *PSECURITY_ATTRIBUTES, *LPSECURITY_ATTRIBUTES;
//----------------------------------------------------------------------------//
//регистры процессора,которые не сохраняются при входе в прерывание
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
}_R4_R11_regs; //8*4=32 байта
#pragma pack()

#pragma pack(1)
typedef struct
{
  HANDLE   ctx;       //контекст потока
  uint32_t flags;     //R2 флаги CF_WAIT_ALL|CF_WAIT_MULTIPLE
  HANDLE*  sync;      //массив списка синхронизации (массив указателей на объекты)
  uint32_t sync_size; //количество объектов синхронизации
  uint32_t wait_ms;   //время ожидания сигнального состояния объектов синхронизации
}SYNC_PAR;
#pragma pack()

//контекст потока
#pragma pack(1)
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //4*4
  //--------coreobj-------<<
  //----mode-------
  //режим,биты: 
  //[0= priv=0/npriv=1]
  //[1= MSP=0/PSP=1]
  //[2= FPU/reserved]для начальной инициализации необходимо установить бит FPU
  //[3..7=уровень приоритета](0-самый приоритетный 31-низкий приоритет)
  //[8-crit section]
  //[9-suspended]
  //[10-set stack]установить размер стека
  uint32_t mode;     
  uint32_t id;  //идентификатор потока (int,положительное)
  //6*4
  //----------------------------------------------
  uint32_t tstack_PSP;    //текущая вершина стека PSP
  uint32_t tstaRAM_PSP;   //начало выделенной области для стека (для контроля и освобождения)
  uint32_t tstsize_PSP;   //размер стека
  //9*4
  uint32_t rsv0;//резервные
  uint32_t rsv1;
  uint32_t rsv2;
  //12*4
  //----------------------------------------------
  uint32_t wait_ms; //всего спать INFINITE=(ULONG)-1-вечный сон либо 
  //время ожидания сигнального состояния объектов синхронизации
  //12 * 4
  //----------------------------------------------
  uint32_t sync_size; //количество объектов синхронизации
  //14 * 4
  //----------------------------------------------
  _R4_R11_regs;  //8 regs=8 * 4 
  //22 x uint32_t  = 88 байтов всего
}TCONTEXT,*PTCONTEXT;
#pragma pack()
typedef TCONTEXT *HTHREAD,*PTHREAD ;

//----------------------------------------------------------------------------//
/*
допускается повторный захват владельцем в любом случае для предотвращения DEADLOCK.
При установленном MF_RECURSIVE ведется счет захватов.
Освобождать 1 раз без рекурсии,либо столько раз,сколько был захвачен.
*/
#define MF_PLAIN      ((uint16_t)CF_PLAIN)       //нет контроля повторного захвата
#define MF_RECURSIVE  ((uint16_t)CF_RECURSIVE)   //повторные захваты владельцем допустимы,ведется счетчик count
#define MF_TIMED      ((uint16_t)CF_TIMED )      //ведется счетчик времени до освобождения захвата

#pragma pack(1)
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //--------coreobj-------<<
  uint32_t  timeout;    //время до сброса захвата(значение загрузки),
  uint32_t  timecount;  //текущее время до сброса захвата,
  uint32_t  count;      //счетчик повторных захватов 
}Mutex;
#pragma pack()
typedef Mutex *HMUTEX,*PMUTEX ;
//----------------------------------------------------------------------------//
#define EF_AUTORESET    ((uint16_t)CF_AUTORESET) //событие с автосбросом (синхрособытие)
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
  uint32_t  maxcount;      //максимальное количество допустимых захватов ;
  uint32_t  count;         //декрементный счетчик допустимых захватов ;
}Semaphore;
#pragma pack()
typedef Semaphore *HSEMAPHORE,*PSEMAPHORE ;
//----------------------------------------------------------------------------//
//системный таймер,общий для всех потоков.Обновление 1 мс.
//исполнение в контексте прерывания.
//системные таймеры,маска характеристик

//таймер с автосбросом при захвате потоком (синхрособытие),по умолчанию 0
#define ST_AUTORESET      ((uint16_t)CF_AUTORESET)  //4==сбрасывать при синхронизации

#define ST_NO_RLD_MSK     ((uint16_t)256)   //не перезагружать при достижении лимита счета (0)
#define ST_DECR_CNT_MSK   ((uint16_t)512)   //тип счетчика - декрементный 
#define ST_DIS_INT_MSK    ((uint16_t)1024)  //запретить вход в функцию прерывавания
#define ST_DIS_CNT_MSK    ((uint16_t)2048)  //запретить счет

#pragma pack(1)
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //--------coreobj-------<<
  uint16_t period;            //период таймера в мС
  uint16_t counter;           //текущий счетчик
  void (*ITFunc)(void )  ;    //функция прерывания
}SystemTimer;
#pragma pack()
typedef SystemTimer *HSYSTIMER,*PSYSTIMER ;

//------system timers macros------
#define _isSysTimerDecr(_pt)              ((_pt)->flags & ST_DECR_CNT_MSK)     //декрементный отсчет интервала
#define _isSysTimerIncr(_pt)              (((_pt)->flags & ST_DECR_CNT_MSK)==0)//инкрементный отсчет интервала

#define _isSysTimerCounterDisabled(_pt)   ((_pt)->flags & ST_DIS_CNT_MSK)      //счет разрешен
#define _isSysTimerCounterEnabled(_pt)    (((_pt)->flags & ST_DIS_CNT_MSK)==0) //счет разрешен

#define _isSysTimerReloadDisabled(_pt)    ((_pt)->flags & ST_NO_RLD_MSK)      //перезагрузка запрещена
#define _isSysTimerReloadEnabled(_pt)     (((_pt)->flags & ST_NO_RLD_MSK)==0) //перезагрузка разрешена

#define _isSysTimerITDisabled(_pt)        ((_pt)->flags & ST_DIS_INT_MSK)      //запрет входа в функцию прерывавания
#define _isSysTimerITEnabled(_pt)         (((_pt)->flags & ST_DIS_INT_MSK)==0) //разрешен вход в функцию прерывавания
//----------------------------------------------------------------------------//
//отложенная задача
/*
может быть сконфигурирована как функция таймера потокового времени исполнения.
В случае numExecutes!=0 ,будет отсчет запусков,после этого будет удалена,иначе 
в случае периодической задачи будет повторяться до удаления.
*/
#define TF_FREE_ARGS 2
#define TF_PERIODIC  4

typedef struct
{
  //--------coreobj------->>
  COREOBJ;                        //+флаги зачистки/исполнения
  //--------coreobj-------<<
  TFUNC* pTFunc;                  //функция задачи int (*pTFunc)(void* pargs) 
  void* pargs;                    //аргументы
  uint32_t period;                 //период исполнения/задержка исполнения
  
  uint32_t nextRun;                //таймштамп следующего исполнения
  uint32_t numExecutes;            //отсчет исполнений
}Task;

typedef Task *HTASK,*PTASK ;
//----------------------------------------------------------------------------//

#endif // __CORE_TYPES_H -----2017 @MW dem1305 EOF