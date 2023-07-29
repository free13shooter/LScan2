/**
  Laser project by dem1305

  @2017
  
  объекты синхронизации
*/
//----------------------------------------------------------------------------//
#include "core.h"
//----------------------------------------------------------------------------//
/*
принцип сихронизации объектов ядра:
объект ядра может иметь ссылку на односвязный список объектов,
к которым он привязан по сигнальному признаку.
Если любой из объектов находится в несигнальном состоянии-поток не может быть 
задействован в следующий квант времени исполнения(не проходит проверку синхронизации),
поэтому будет пропущен планировщиком.
*/
//----------------------------------------------------------------------------//
/*
Создать мьютекс,в случае успеха вернет дескриптор.
Если не удалось создать,вернет NULL

Ограничений на захват мьютекса владельцем нет.
Если мьютекс рекурсивный-необходимо освобождать его соттветствующее количество раз.
  флаги : 
  MF_PLAIN      ==0-нет контроля повторного захвата
  MF_RECURSIVE  ==1-повторные захваты владельцем допустимы,ведется счетчик count
  MF_TIMED      ==2-ведется счетчик времени до освобождения захвата
*/
HANDLE CreateMutex(uint16_t flagsMask,BOOL bInitialOwner,uint32_t timeout)
{
  _save_op(13);
  
  Mutex* hM=(pctcon->mode & TMMRAM_STACK_Msk)?mmmalloc(sizeof(Mutex)):ccmalloc(sizeof(Mutex));
  if(hM==0)return NULL;
  hM->_ty=(uint16_t)TY_MUTEX;
  hM->sync=(HANDLE*)(bInitialOwner?CurrActiveThreadCTX:0);//захват
  hM->flags=(bInitialOwner?CF_PASSIVE:CF_SIGNAL)|flagsMask;
  hM->timecount=hM->timeout=(flagsMask & MF_TIMED)?timeout:INFINITE;    //время до сброса захвата,
  hM->count=bInitialOwner?1:0;   //счетчик повторных захватов
  hM->prev=hM->next=0;
  
  //---------------в прерывании ?------------------
  if(check_if_IT_less_or_equal())
  {
    return (HANDLE)core_add_CoreObj((PCOREOBJ)hM,0);
  }
  else
  {
    if(call_add_core_obj((PCOREOBJ)hM,CurrActiveThreadCTX)==(PCOREOBJ)hM)
    {
      return (HANDLE)hM;
    }
  }
  
  mfree(hM);
  return 0;
}
//----------------------------------------------------------------------------//
//Сканирование объектов ядра каждый тик планировщика.
//Рекурсивные мьютексы с таймаутом освобождаются нужное количество раз с таймаутами.

HTHREAD _ts_scan_core_obj()
{
  _save_op(14);
  
  HTHREAD ht;Mutex* pmu;
  SystemTimer* pst;
  uint8_t pth=0,pthcurr=0;
  uint16_t u16val;
  HTHREAD pendingThread=0;
  
  uint32_t it=check_if_IT_less_or_equal();
    
  for(PCOREOBJ pco=CoreObjListAnchor;pco!=0 ;pco=(PCOREOBJ)pco->next)
  {
    switch(pco->_ty)
    {
    //----------TY_MUTEX--------------------------
    case TY_MUTEX:
      pmu=(Mutex*)pco;
      
      if((pmu->flags & MF_TIMED)) 
      {
        if(pmu->flags & CF_SIGNAL){pmu->timecount=pmu->timeout;break;}
        //счет времени только в несигнальном состоянии
        else if(pmu->timecount!=0 && pmu->timeout!=INFINITE)
        {
          pmu->timecount--;
          if(pmu->timecount==0)
          {
            pmu->timecount=pmu->timeout;
            pmu->count--;//(MF_TIMED!), нерекурсивный имеет только одну ссылку.
            if(pmu->count==0)
            {
              pmu->flags|=(uint16_t)CF_SIGNAL;
              pmu->flags&=~(uint16_t)CF_ABANDONED;
              pmu->sync=0;
            }
          }
        }
      }
      break;
    //----------TY_THREAD--------------------------
    case TY_THREAD:

      ht=(HTHREAD)pco;
      
      if(ht->wait_ms!=INFINITE && ht->wait_ms>0)//ожидание синхронизации
      {
        ht->wait_ms--;
        if(ht->wait_ms==0)ht->flags|=CF_TIMEOUT;
      }
      //проверка ожидающих потоков
#if ENABLE_MULTITHREADING==1      
      pth= _tprio(ht->mode);
      pthcurr= _tprio(CurrActiveThreadCTX->mode);
      
      if(CurrActiveThreadCTX!=ht && ht->sync!=0 && ( pth > pthcurr) )
      {
        //-------запуск ожидающего потока с приоритетом выше----------
        if((CurrActiveThreadCTX->mode & (TCRITSECT_Msk|TSUSPENDED_Msk))==0 && _check_for_sync(ht)==ht)
        {
          pendingThread=ht;
        }
      }
#endif
      break;
    
    //----------SYSTIMER--------------------------  
    case TY_SYSTIMER:  
      pst=(HSYSTIMER)pco;
      
      if(pst->period==0)continue;//таймер не настроен или не назначен
    
      if(_isSysTimerCounterEnabled(pst))//счет разрешен
      {
        if(_isSysTimerDecr(pst))pst->counter--;else pst->counter++;
      }
      else break;//к следующему
    
      u16val=pst->period;
    
      if( (_isSysTimerDecr(pst) && pst->counter==0) || (_isSysTimerIncr(pst) && pst->counter==u16val) )
      {
        pst->flags|=(uint16_t)CF_SIGNAL;
        
        if(_isSysTimerReloadEnabled(pst))//разрешена перезагрузка
        {
          if(_isSysTimerDecr(pst))pst->counter=u16val;//decrementive=reload
          else pst->counter=0;//incrementive=0
        }
        
        if(_isSysTimerITEnabled(pst))//флаг запрета не установлен
        {
          if(pst->ITFunc!=0)pst->ITFunc();//вызвать функцию прерывания
          if(_isSysTimerReloadDisabled(pst))
          {
            pst->flags |= (uint8_t)(ST_DIS_INT_MSK | ST_DIS_CNT_MSK);//запретить повторный вызов и счет
          }
        }
      }
  
    break;
    //----------TY_EVENT--------------------------
    case TY_EVENT:
    //--------------------------------------------
    default:
      if(pco->_ty > TY_MAX)//crit error in core
      {
        //int n=10000000;
        //HTHREAD htr=pctcon;
        //uint32_t id=htr->id;
        while (1);//n--);
      }
      break;
    }
    
    PCOREOBJ core_end=_pend_;
    if(pco==core_end)break;
  }
  
  return pendingThread;
}
//----------------------------------------------------------------------------//
/*
Выполняется в прерывании уровня ядра,в функции выбора контекста для переключения,
до проверки нахождения в критической секции текущего потока,
Проверка таймера ожидания wait_ms (используется в Sleep и ожидании объектов).

Проверяет на привязку к объектам синхронизации.
В случае,если синхронизация не пройдена(сигнальное состояние невозможно)-вернет 0,
иначе хендл потока
вход: R0 содержит контекст для проверки.

В sync_size потока записывается индекс объекта,удовлетворивший условие,
или младший индекс из списка объектов
*/

HTHREAD _check_for_sync(HTHREAD ht)
{
  _save_op(15);
  
  uint32_t o=0;
  Mutex* pmu;
  Semaphore* ps;
  Event* pe;
  SystemTimer* pti;
  
  uint32_t it=check_if_IT_less_or_equal();
  //--------------сканирование------------>>
    
  for(o=0;ht->sync!=0 && o<ht->sync_size;o++)
  {
    PCOREOBJ pco=ht->sync_size==1?(PCOREOBJ)ht->sync:(PCOREOBJ)ht->sync[o];
    
    if(pco==0)continue;//dbg_error_led(ERR_ZERO_PTR);//deleted !
    
    switch(pco->_ty)
    {
    case TY_MUTEX:
      //поиск владельца мьютекса
      if(pco->_ty==TY_MUTEX && pco->sync && ((pco->flags & CF_ABANDONED)==0))//уже обнаружен?
      {
        pmu=(Mutex*)pco;
        if(core_find_CoreObj((PCOREOBJ)pmu->sync))goto check_state_sync;//найден
        
           //обнаружен брошенный мьютекс
        ht->flags|=(uint16_t)(CF_ABANDONED);//уже обнаружен
        pmu->sync=(HANDLE*)ht;//владелец
        pmu->count=1;//единственный
        pmu->flags &=((uint16_t)~(CF_SIGNAL))|(uint16_t)CF_ABANDONED;
      }
      break;
    
    
    default://событие,поток,семафор проверяются только на сигнальное состояние
      break;
    }
    
  check_state_sync:
    //в сигнальном состоянии либо захваченный мьютекс
    if((pco->flags & CF_SIGNAL)||(pco->_ty==TY_MUTEX && pco->sync==(HANDLE*)ht))
    {
      //wait one
      if((CF_WAIT_ALL & ht->flags)==0)goto out_res_ch_sync;//wait one.готов.
      //wait all
      if(o==ht->sync_size-1)goto out_res_ch_sync;//все в сигнальном состоянии
      else continue;//продолжить скан
    }
    
    if(CF_WAIT_ALL & ht->flags) return 0;//несигнальное состояние.
  }
  
  //-------Sleep не пройден или таймаут?------------
  if(ht->wait_ms && ((ht->flags& CF_TIMEOUT)==0) )return 0;//0=завершено(TIMEOUT),иначе не 0(может быть INFINITE)
  else 
  {
    if(ht->flags & CF_TIMEOUT)o=0;
    goto out_res_clear_sync;//без ожидания
  }
  //------------------------------------------------
out_res_ch_sync:
  //------захват объекта/всех объектов-------
  if(CF_WAIT_ALL & ht->flags)o=0;
    
  for(;ht->sync!=0 && o<ht->sync_size;o++)
  {
    PCOREOBJ pco=ht->sync_size==1?(PCOREOBJ)ht->sync:(PCOREOBJ)ht->sync[o];
    
    switch(pco->_ty)
    {
    case TY_MUTEX:
      pmu=(Mutex*)pco;
      pmu->flags &=(uint16_t)~(CF_SIGNAL|CF_ABANDONED);
      pmu->sync=(HANDLE*)ht;
      if(pmu->flags&CF_RECURSIVE){if(pmu->count<MAX_UINT32T)pmu->count++;}
       else pmu->count=1;
      break;
    //----------
    case TY_SEMAPHORE:
      ps=(Semaphore*)pco;
      //поиск свободного
      for(uint32_t i=0;i<ps->maxcount && ps->count>0;i++)//счетчик позволяет
      {
        if(ps->sync[i]==0)
        {
          ps->sync[i]=(HANDLE)ht;
          ps->count--;
          if(ps->count==0)ps->flags &=(uint16_t)~(CF_SIGNAL|CF_ABANDONED);
          goto out_res_clear_sync;
        }
      }
      
      dbg_error_led(ERR_NOT_FOUND);//deleted !
      break;
    //----------
    case TY_EVENT://проверка на автосброс
      pe=(Event*)pco;
      if(pe->flags&EF_AUTORESET)pe->flags &=(uint16_t)~(CF_SIGNAL);
      break;
    //----------
    case TY_SYSTIMER://проверка на автосброс
      pti=(SystemTimer*)pco;
      if(pti->flags & ST_AUTORESET)pti->flags &=(uint16_t)~(CF_SIGNAL);
      break;  
    //----------  
    default:
      break;
    }
    
    if((CF_WAIT_ALL & ht->flags)==0)goto out_res_clear_sync;//только один (o)
  }
  
  if(CF_WAIT_ALL & ht->flags)o=0;
  
out_res_clear_sync:
  //---------------------------------  
  ht->wait_ms=0;//условие удовлетворяет,очиска таймера ожидания.
  //очистить зависимости и запомнить индекс последнего проверяемого объекта
  ht->sync=0;
  ht->sync_size=o;
  return ht;
}
//----------------------------------------------------------------------------//
/*
Ожидание сигнального состояния массива объектов синхронизации.
В режиме ожидания поток не занимает время процессора.

По завершению ожидания в sync_size содержится индекс объекта,удовлетворивший условие,
или младший индекс из списка объектов,sync_size обнуляется на выходе функции.
Индекс суммируется с выходным статусом ожидания (значение возврата функции).
*/
DWORD WaitForMultipleObjects(
  _In_       DWORD  nCount,         //количество объектов синхронизации
  _In_ const HANDLE *lpHandles,     //массив  объектов синхронизации
  _In_       BOOL   bWaitAll,       //ожидать все объекты?(1=все,0=любой один)
  _In_       DWORD  dwMilliseconds  //время ожидания ( 0=выход без ожидания !)
)
{
  _save_op(16);
  //ожидание в прерывании запрещено
  if(check_if_IT_less_or_equal())
  {
    return WAIT_FAILED;//while(1);
  }
  //-------------------------------
  if(lpHandles==0||nCount==0||nCount>MAXIMUM_WAIT_OBJECTS)return WAIT_FAILED;//error in parameters
  if(dwMilliseconds==0)return STATUS_WAIT_0;
  
  SYNC_PAR sp=
  {
    0,                    //контекст потока
    ((bWaitAll?(uint32_t)CF_WAIT_ALL:(uint32_t)0)|(uint32_t)CF_WAIT_MULTIPLE),   //флаги
    (HANDLE*)lpHandles,   //список объектов синхронизации
    nCount,
    dwMilliseconds        //время ожидания сигнального состояния объектов синхронизации
  };
  
  call_save_thread_sync_parameters(&sp);//записать массив в sync потока
  //SCB->ICSR|=SCB_ICSR_PENDSVSET_Msk;//SW

  _core_sw_thread;

  //------------------------------------
  DWORD exret=STATUS_WAIT_0;
  
  uint16_t flags= CurrActiveThreadCTX->flags;
  if(flags & CF_ABANDONED )exret|=WAIT_ABANDONED;
    else if( flags & CF_TIMEOUT )exret=WAIT_TIMEOUT;
  if(exret==STATUS_WAIT_0 || exret==WAIT_ABANDONED)
    exret+=(DWORD)CurrActiveThreadCTX->sync_size; //индекс из списка объектов 
  
  return exret;
}
//----------------------------------------------------------------------------//
/*
Ожидание сигнального состояния объекта синхронизации.
В режиме ожидания поток не занимает время процессора.

По завершению функция вернет статус ожидания.
*/
DWORD WaitForSingleObject(
  _In_ HANDLE hHandle,        //объект синхронизации
  _In_ DWORD  dwMilliseconds  //время ожидания ( 0=выход без ожидания !)
)
{
  _save_op(17);
  //ожидание в прерывании запрещено
  if(check_if_IT_less_or_equal())
  {
    return WAIT_FAILED;//while (1);
  }
  //-------------------------------
  if(hHandle==0)return WAIT_FAILED;//error in parameters
  if(dwMilliseconds==0)return STATUS_WAIT_0;
  
  SYNC_PAR sp=
  {
    0,                //контекст потока
    0,                //флаги
    (HANDLE*)hHandle, //список синхронизации
    1,
    dwMilliseconds    //время ожидания сигнального состояния объектов синхронизации
  };
  
  HTHREAD ht=CurrActiveThreadCTX;
  call_save_thread_sync_parameters(&sp);

  _core_sw_thread;

  //------------------------------------
  DWORD exret=STATUS_WAIT_0;
  uint16_t flags= CurrActiveThreadCTX->flags;
  
  if(flags & CF_ABANDONED)exret|=WAIT_ABANDONED;
    else if(flags & CF_TIMEOUT)exret=WAIT_TIMEOUT;
  
  return exret;
}
//----------------------------------------------------------------------------//
BOOL ReleaseMutex( _In_ HANDLE hMutex )//освобождает один раз
{
  _save_op(18);
  
  BOOL resu;
  
  if(check_if_IT_less_or_equal())
  {
    if(hMutex==0 || !core_find_CoreObj((PCOREOBJ) hMutex) ||
        ((PCOREOBJ)hMutex)->_ty!=TY_MUTEX)return FALSE;
    
    resu= _rls_object(hMutex,0);
  }
  else
  {
    if(hMutex==0 ||((PCOREOBJ)hMutex)->_ty!=TY_MUTEX)return FALSE;
    resu=call_rls_object((PCOREOBJ)hMutex,CurrActiveThreadCTX);
  }
  
  return resu;
}
//----------------------------------------------------------------------------//
//освобождение объекта.
BOOL _rls_object(HANDLE pco_R0,HTHREAD ht_R1)
{
  _save_op(19);
  
  uint32_t it=check_if_IT_less_or_equal();
  
  Mutex* pmu;Semaphore* ps;
  
  HTHREAD ht=ht_R1?ht_R1:CurrActiveThreadCTX;
  
  //---------------
  switch(((PCOREOBJ)pco_R0)->_ty)
    {
    case TY_MUTEX:
      pmu=(Mutex*)pco_R0;
      if(pmu->sync!=(HANDLE*)ht || (pmu->count ==0) || (pmu->flags & CF_SIGNAL))return 0;//not owner==error
      if(pmu->flags & MF_RECURSIVE)pmu->count--;else pmu->count=0;//нет контроля
      
      if(pmu->count ==0)
      {
        pmu->sync=0;
        pmu->flags &=(uint16_t)~(CF_ABANDONED);
        pmu->flags|=CF_SIGNAL;
        pmu->timecount=pmu->timeout;
        break;
      }
      break;
    //----------
    case TY_SEMAPHORE:  
      ps=(Semaphore*)pco_R0;
      for(uint32_t i=0;i<ps->maxcount;i++)
      {
        if(ps->sync[i]==(HANDLE)ht && ps->count<ps->maxcount)
        {
          ps->sync[i]=0;ps->count++;
          if(ps->count>0)ps->flags|=(uint16_t) CF_SIGNAL;
        }
      }
      return 0;//not found==error
      break;
    //----------
    default:
      return 0;//error
      break;
    }
    //------------------------
  
  return TRUE;
}
//----------------------------------------------------------------------------//
/*
Создать событие,в случае успеха вернет дескриптор.
Если не удалось создать,вернет NULL
*/
HANDLE CreateEvent 
(
_In_ BOOL bManualReset,	// тип сброса TRUE=ручной
_In_ BOOL bInitialState	// начальное состояние TRUE=сигнальное
)
{
  _save_op(20);
  
  Event* he=(pctcon->mode & TMMRAM_STACK_Msk)?mmmalloc(sizeof(Event)):ccmalloc(sizeof(Event));
  if(he==0)return NULL;
  he->_ty=(uint16_t)TY_EVENT;
  he->sync=(HANDLE*)CurrActiveThreadCTX;//владелец
  he->flags=(uint16_t)(bInitialState?CF_SIGNAL:CF_PASSIVE);
  if(!bManualReset)he->flags|=(uint16_t)CF_AUTORESET;
  he->prev=he->next=0;
  
  //---------------в прерывании ?------------------
  if(check_if_IT_less_or_equal())
  {
    return (HANDLE)core_add_CoreObj((PCOREOBJ)he,0);
  }
  else
  {
    if(call_add_core_obj((PCOREOBJ)he,CurrActiveThreadCTX)==(PCOREOBJ)he)
    {
      return (HANDLE)he;
    }
  }
  //-----------------------------------------------
  
  //неудача
  mfree(he);
  return 0;
  
}
//установить событие в сигнальное состояние.В случае успеха вернет не 0.
BOOL SetEvent(HANDLE hEvent)
{
  _save_op(21);
  
  if(check_if_IT_less_or_equal())
  {  
    if(hEvent==0 || !core_find_CoreObj((PCOREOBJ) hEvent) ||
        ((PCOREOBJ)hEvent)->_ty!=TY_EVENT)return FALSE;
    
    ((PCOREOBJ)hEvent)->flags|=(uint16_t)CF_SIGNAL;
    return TRUE;
  }
  else
  {
    if(hEvent==0 ||((PCOREOBJ)hEvent)->_ty!=TY_EVENT)return FALSE;
    
    if(call_change_core_object_flags((PCOREOBJ)hEvent,(uint16_t)CF_SIGNAL,(uint8_t)SET,CurrActiveThreadCTX))
    return TRUE;
  }
  
  return FALSE;
}
//установить событие в несигнальное состояние.В случае успеха вернет не 0.
BOOL ResetEvent(HANDLE hEvent)
{
  _save_op(22);
  
  if(check_if_IT_less_or_equal())
  {
    if(hEvent==0 || !core_find_CoreObj((PCOREOBJ) hEvent) ||
        ((PCOREOBJ)hEvent)->_ty!=TY_EVENT)return FALSE;
    
    ((PCOREOBJ)hEvent)->flags|=(uint16_t)CF_SIGNAL;
  }
  else
  {
    if(hEvent==0 ||((PCOREOBJ)hEvent)->_ty!=TY_EVENT)return FALSE;
    
    if(call_change_core_object_flags((PCOREOBJ)hEvent,(uint16_t)CF_SIGNAL,(uint8_t)RESET,CurrActiveThreadCTX))
    return TRUE;
  }
  
  return FALSE;
}
//----------------------------------------------------------------------------//
/*
Создать семафор,в случае успеха вернет дескриптор.
Если не удалось создать,вернет NULL
*/
HANDLE CreateSemaphore(
  _In_     LONG                  lInitialCount, //инициализация счетчика доступов
  _In_     LONG                  lMaximumCount  //максимально доступов
)
{
  _save_op(23);
  
  if(lMaximumCount==0 || lInitialCount==0)return NULL;
  
  Semaphore* hs=(pctcon->mode & TMMRAM_STACK_Msk)?mmmalloc(sizeof(Semaphore)):ccmalloc(sizeof(Semaphore));
  if(hs==0)return NULL;
  
  hs->sync=(pctcon->mode & TMMRAM_STACK_Msk)?mmmalloc(lMaximumCount*sizeof(HANDLE)):ccmalloc(lMaximumCount*sizeof(HANDLE));
  if(hs->sync==0){mfree(hs);return NULL;}
  
  ZeroMemory(hs->sync,lMaximumCount*sizeof(HANDLE));
  hs->_ty=(uint16_t)TY_SEMAPHORE;
  hs->flags=(uint16_t)CF_SIGNAL;
  hs->prev=hs->next=0;
  hs->count=lInitialCount>lMaximumCount?lMaximumCount:lInitialCount;
  hs->maxcount=lMaximumCount;
  
  if(check_if_IT_less_or_equal())
  {
    return (HANDLE)core_add_CoreObj((PCOREOBJ)hs,0);
  }
  else
  {
    if(call_add_core_obj((PCOREOBJ)hs,CurrActiveThreadCTX)==(PCOREOBJ)hs)
    {
      return (HANDLE)hs;
    }
  }
  mfree(hs->sync);
  mfree(hs);
  return 0;
  
}

BOOL ReleaseSemaphore( _In_ HANDLE hs )//освобождает один раз
{
  _save_op(24);
  
  BOOL resu;
  
  if(check_if_IT_less_or_equal())
  {
    
    if(hs==0 || !core_find_CoreObj((PCOREOBJ) hs) ||
        ((PCOREOBJ)hs)->_ty!=TY_SEMAPHORE)return FALSE;
    
    resu= _rls_object(hs,0);
  }
  else
  {
    if(hs==0 ||((PCOREOBJ)hs)->_ty!=TY_EVENT)return FALSE;
    resu=call_rls_object((PCOREOBJ)hs,CurrActiveThreadCTX);
  }
  
  return resu;
}
//----------------------------------------------------------------------------//