/**
  Laser project by dem1305

  @2017
  
  ������� �������������
*/
//----------------------------------------------------------------------------//
#include "core.h"
//----------------------------------------------------------------------------//
/*
������� ������������ �������� ����:
������ ���� ����� ����� ������ �� ����������� ������ ��������,
� ������� �� �������� �� ����������� ��������.
���� ����� �� �������� ��������� � ������������ ���������-����� �� ����� ���� 
������������ � ��������� ����� ������� ����������(�� �������� �������� �������������),
������� ����� �������� �������������.
*/
//----------------------------------------------------------------------------//
/*
������� �������,� ������ ������ ������ ����������.
���� �� ������� �������,������ NULL

����������� �� ������ �������� ���������� ���.
���� ������� �����������-���������� ����������� ��� ��������������� ���������� ���.
  ����� : 
  MF_PLAIN      ==0-��� �������� ���������� �������
  MF_RECURSIVE  ==1-��������� ������� ���������� ���������,������� ������� count
  MF_TIMED      ==2-������� ������� ������� �� ������������ �������
*/
HANDLE CreateMutex(uint16_t flagsMask,BOOL bInitialOwner,uint32_t timeout)
{
  _save_op(13);
  
  Mutex* hM=(pctcon->mode & TMMRAM_STACK_Msk)?mmmalloc(sizeof(Mutex)):ccmalloc(sizeof(Mutex));
  if(hM==0)return NULL;
  hM->_ty=(uint16_t)TY_MUTEX;
  hM->sync=(HANDLE*)(bInitialOwner?CurrActiveThreadCTX:0);//������
  hM->flags=(bInitialOwner?CF_PASSIVE:CF_SIGNAL)|flagsMask;
  hM->timecount=hM->timeout=(flagsMask & MF_TIMED)?timeout:INFINITE;    //����� �� ������ �������,
  hM->count=bInitialOwner?1:0;   //������� ��������� ��������
  hM->prev=hM->next=0;
  
  //---------------� ���������� ?------------------
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
//������������ �������� ���� ������ ��� ������������.
//����������� �������� � ��������� ������������� ������ ���������� ��� � ����������.

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
        //���� ������� ������ � ������������ ���������
        else if(pmu->timecount!=0 && pmu->timeout!=INFINITE)
        {
          pmu->timecount--;
          if(pmu->timecount==0)
          {
            pmu->timecount=pmu->timeout;
            pmu->count--;//(MF_TIMED!), ������������� ����� ������ ���� ������.
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
      
      if(ht->wait_ms!=INFINITE && ht->wait_ms>0)//�������� �������������
      {
        ht->wait_ms--;
        if(ht->wait_ms==0)ht->flags|=CF_TIMEOUT;
      }
      //�������� ��������� �������
#if ENABLE_MULTITHREADING==1      
      pth= _tprio(ht->mode);
      pthcurr= _tprio(CurrActiveThreadCTX->mode);
      
      if(CurrActiveThreadCTX!=ht && ht->sync!=0 && ( pth > pthcurr) )
      {
        //-------������ ���������� ������ � ����������� ����----------
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
      
      if(pst->period==0)continue;//������ �� �������� ��� �� ��������
    
      if(_isSysTimerCounterEnabled(pst))//���� ��������
      {
        if(_isSysTimerDecr(pst))pst->counter--;else pst->counter++;
      }
      else break;//� ����������
    
      u16val=pst->period;
    
      if( (_isSysTimerDecr(pst) && pst->counter==0) || (_isSysTimerIncr(pst) && pst->counter==u16val) )
      {
        pst->flags|=(uint16_t)CF_SIGNAL;
        
        if(_isSysTimerReloadEnabled(pst))//��������� ������������
        {
          if(_isSysTimerDecr(pst))pst->counter=u16val;//decrementive=reload
          else pst->counter=0;//incrementive=0
        }
        
        if(_isSysTimerITEnabled(pst))//���� ������� �� ����������
        {
          if(pst->ITFunc!=0)pst->ITFunc();//������� ������� ����������
          if(_isSysTimerReloadDisabled(pst))
          {
            pst->flags |= (uint8_t)(ST_DIS_INT_MSK | ST_DIS_CNT_MSK);//��������� ��������� ����� � ����
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
����������� � ���������� ������ ����,� ������� ������ ��������� ��� ������������,
�� �������� ���������� � ����������� ������ �������� ������,
�������� ������� �������� wait_ms (������������ � Sleep � �������� ��������).

��������� �� �������� � �������� �������������.
� ������,���� ������������� �� ��������(���������� ��������� ����������)-������ 0,
����� ����� ������
����: R0 �������� �������� ��� ��������.

� sync_size ������ ������������ ������ �������,��������������� �������,
��� ������� ������ �� ������ ��������
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
  //--------------������������------------>>
    
  for(o=0;ht->sync!=0 && o<ht->sync_size;o++)
  {
    PCOREOBJ pco=ht->sync_size==1?(PCOREOBJ)ht->sync:(PCOREOBJ)ht->sync[o];
    
    if(pco==0)continue;//dbg_error_led(ERR_ZERO_PTR);//deleted !
    
    switch(pco->_ty)
    {
    case TY_MUTEX:
      //����� ��������� ��������
      if(pco->_ty==TY_MUTEX && pco->sync && ((pco->flags & CF_ABANDONED)==0))//��� ���������?
      {
        pmu=(Mutex*)pco;
        if(core_find_CoreObj((PCOREOBJ)pmu->sync))goto check_state_sync;//������
        
           //��������� ��������� �������
        ht->flags|=(uint16_t)(CF_ABANDONED);//��� ���������
        pmu->sync=(HANDLE*)ht;//��������
        pmu->count=1;//������������
        pmu->flags &=((uint16_t)~(CF_SIGNAL))|(uint16_t)CF_ABANDONED;
      }
      break;
    
    
    default://�������,�����,������� ����������� ������ �� ���������� ���������
      break;
    }
    
  check_state_sync:
    //� ���������� ��������� ���� ����������� �������
    if((pco->flags & CF_SIGNAL)||(pco->_ty==TY_MUTEX && pco->sync==(HANDLE*)ht))
    {
      //wait one
      if((CF_WAIT_ALL & ht->flags)==0)goto out_res_ch_sync;//wait one.�����.
      //wait all
      if(o==ht->sync_size-1)goto out_res_ch_sync;//��� � ���������� ���������
      else continue;//���������� ����
    }
    
    if(CF_WAIT_ALL & ht->flags) return 0;//������������ ���������.
  }
  
  //-------Sleep �� ������� ��� �������?------------
  if(ht->wait_ms && ((ht->flags& CF_TIMEOUT)==0) )return 0;//0=���������(TIMEOUT),����� �� 0(����� ���� INFINITE)
  else 
  {
    if(ht->flags & CF_TIMEOUT)o=0;
    goto out_res_clear_sync;//��� ��������
  }
  //------------------------------------------------
out_res_ch_sync:
  //------������ �������/���� ��������-------
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
      //����� ����������
      for(uint32_t i=0;i<ps->maxcount && ps->count>0;i++)//������� ���������
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
    case TY_EVENT://�������� �� ���������
      pe=(Event*)pco;
      if(pe->flags&EF_AUTORESET)pe->flags &=(uint16_t)~(CF_SIGNAL);
      break;
    //----------
    case TY_SYSTIMER://�������� �� ���������
      pti=(SystemTimer*)pco;
      if(pti->flags & ST_AUTORESET)pti->flags &=(uint16_t)~(CF_SIGNAL);
      break;  
    //----------  
    default:
      break;
    }
    
    if((CF_WAIT_ALL & ht->flags)==0)goto out_res_clear_sync;//������ ���� (o)
  }
  
  if(CF_WAIT_ALL & ht->flags)o=0;
  
out_res_clear_sync:
  //---------------------------------  
  ht->wait_ms=0;//������� �������������,������ ������� ��������.
  //�������� ����������� � ��������� ������ ���������� ������������ �������
  ht->sync=0;
  ht->sync_size=o;
  return ht;
}
//----------------------------------------------------------------------------//
/*
�������� ����������� ��������� ������� �������� �������������.
� ������ �������� ����� �� �������� ����� ����������.

�� ���������� �������� � sync_size ���������� ������ �������,��������������� �������,
��� ������� ������ �� ������ ��������,sync_size ���������� �� ������ �������.
������ ����������� � �������� �������� �������� (�������� �������� �������).
*/
DWORD WaitForMultipleObjects(
  _In_       DWORD  nCount,         //���������� �������� �������������
  _In_ const HANDLE *lpHandles,     //������  �������� �������������
  _In_       BOOL   bWaitAll,       //������� ��� �������?(1=���,0=����� ����)
  _In_       DWORD  dwMilliseconds  //����� �������� ( 0=����� ��� �������� !)
)
{
  _save_op(16);
  //�������� � ���������� ���������
  if(check_if_IT_less_or_equal())
  {
    return WAIT_FAILED;//while(1);
  }
  //-------------------------------
  if(lpHandles==0||nCount==0||nCount>MAXIMUM_WAIT_OBJECTS)return WAIT_FAILED;//error in parameters
  if(dwMilliseconds==0)return STATUS_WAIT_0;
  
  SYNC_PAR sp=
  {
    0,                    //�������� ������
    ((bWaitAll?(uint32_t)CF_WAIT_ALL:(uint32_t)0)|(uint32_t)CF_WAIT_MULTIPLE),   //�����
    (HANDLE*)lpHandles,   //������ �������� �������������
    nCount,
    dwMilliseconds        //����� �������� ����������� ��������� �������� �������������
  };
  
  call_save_thread_sync_parameters(&sp);//�������� ������ � sync ������
  //SCB->ICSR|=SCB_ICSR_PENDSVSET_Msk;//SW

  _core_sw_thread;

  //------------------------------------
  DWORD exret=STATUS_WAIT_0;
  
  uint16_t flags= CurrActiveThreadCTX->flags;
  if(flags & CF_ABANDONED )exret|=WAIT_ABANDONED;
    else if( flags & CF_TIMEOUT )exret=WAIT_TIMEOUT;
  if(exret==STATUS_WAIT_0 || exret==WAIT_ABANDONED)
    exret+=(DWORD)CurrActiveThreadCTX->sync_size; //������ �� ������ �������� 
  
  return exret;
}
//----------------------------------------------------------------------------//
/*
�������� ����������� ��������� ������� �������������.
� ������ �������� ����� �� �������� ����� ����������.

�� ���������� ������� ������ ������ ��������.
*/
DWORD WaitForSingleObject(
  _In_ HANDLE hHandle,        //������ �������������
  _In_ DWORD  dwMilliseconds  //����� �������� ( 0=����� ��� �������� !)
)
{
  _save_op(17);
  //�������� � ���������� ���������
  if(check_if_IT_less_or_equal())
  {
    return WAIT_FAILED;//while (1);
  }
  //-------------------------------
  if(hHandle==0)return WAIT_FAILED;//error in parameters
  if(dwMilliseconds==0)return STATUS_WAIT_0;
  
  SYNC_PAR sp=
  {
    0,                //�������� ������
    0,                //�����
    (HANDLE*)hHandle, //������ �������������
    1,
    dwMilliseconds    //����� �������� ����������� ��������� �������� �������������
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
BOOL ReleaseMutex( _In_ HANDLE hMutex )//����������� ���� ���
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
//������������ �������.
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
      if(pmu->flags & MF_RECURSIVE)pmu->count--;else pmu->count=0;//��� ��������
      
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
������� �������,� ������ ������ ������ ����������.
���� �� ������� �������,������ NULL
*/
HANDLE CreateEvent 
(
_In_ BOOL bManualReset,	// ��� ������ TRUE=������
_In_ BOOL bInitialState	// ��������� ��������� TRUE=����������
)
{
  _save_op(20);
  
  Event* he=(pctcon->mode & TMMRAM_STACK_Msk)?mmmalloc(sizeof(Event)):ccmalloc(sizeof(Event));
  if(he==0)return NULL;
  he->_ty=(uint16_t)TY_EVENT;
  he->sync=(HANDLE*)CurrActiveThreadCTX;//��������
  he->flags=(uint16_t)(bInitialState?CF_SIGNAL:CF_PASSIVE);
  if(!bManualReset)he->flags|=(uint16_t)CF_AUTORESET;
  he->prev=he->next=0;
  
  //---------------� ���������� ?------------------
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
  
  //�������
  mfree(he);
  return 0;
  
}
//���������� ������� � ���������� ���������.� ������ ������ ������ �� 0.
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
//���������� ������� � ������������ ���������.� ������ ������ ������ �� 0.
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
������� �������,� ������ ������ ������ ����������.
���� �� ������� �������,������ NULL
*/
HANDLE CreateSemaphore(
  _In_     LONG                  lInitialCount, //������������� �������� ��������
  _In_     LONG                  lMaximumCount  //����������� ��������
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

BOOL ReleaseSemaphore( _In_ HANDLE hs )//����������� ���� ���
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