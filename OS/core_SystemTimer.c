/**
  ******************************************************************************
  * @author  MW dem1305
  * @version V1.0.0
  * @date    24-Yun-2017
  * @note    ��������� �������,����������� � ��������� 
  *           ���������� ���������� ������� (1 �� ��������)
  ******************************************************************************
  */ 

//----------------------------------------------------------------------------//
#include "core.h"
#include <stdlib.h>
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
/*
typedef struct
{
  //--------coreobj------->>
  COREOBJ;
  //--------coreobj-------<<
  uint16_t period;            //������ ������� � ��
  uint16_t counter;           //������� �������
  
  void (*ITFunc)(void )  ;    //������� ����������
}
SystemTimer;

typedef SystemTimer *HSYSTIMER,*PSYSTIMER ;
*/
//----------------------------------------------------------------------------//
BOOL IsSystemTimer(HANDLE hTimer)//�������� �� ��������� ���������� �� ��������� ������
{
  //-----------� ���������� ?-------->>>
  if(check_if_IT_less_or_equal())
  {
    if(hTimer==0 || !core_find_CoreObj((PCOREOBJ) hTimer) ||
        ((PCOREOBJ)hTimer)->_ty!=TY_SYSTIMER)return FALSE;
    
    return TRUE;
  }
  else
  {
    return call_IsSystemTimer(hTimer);
  }
  //---------------------------------<<<
}
//----------------------------------------------------------------------------//
static BOOL core_is_system_timer(HANDLE pti) 
{
  if(pti!=NULL && core_find_CoreObj((PCOREOBJ)pti) && ((PCOREOBJ)pti)->_ty==TY_SYSTIMER)
    return TRUE;
  
  return FALSE;
}
//----------------------------------------------------------------------------//
//�������� �� ��������� ���������� �� ��������� ������
void svc_isSystemTimer(HANDLE hTimer) 
{
  if(core_is_system_timer(hTimer))__out_R0_to_current_stack_ctx(TRUE);
  else __out_R0_to_stack_R1_ctx(FALSE,pctcon);
}

//----------------------------------------------------------------------------//
//���������� ������������ ������ � �������������.������� ��������� ������� � ������� �������� ��� NULL � ������ �������.
//���������- ������,��������� �� ������� ��������� ������ ����������
SystemTimer* SetPeriodicSystemTimer(uint16_t periodMs,void (*ITFunc)(void ))
{
  return SetSystemTimer(periodMs,ITFunc,0);
}
//----------------------------------------------------------------------------//
//���������� ������������ ������ ��� ������������.������� ��������� ������� � ������� �������� ��� NULL � ������ �������.
//���������- ������,��������� �� ������� ��������� ������ ����������
SystemTimer* SetOnePulseSystemTimer(uint16_t periodMs,void (*ITFunc)(void ))
{
  return SetSystemTimer(periodMs,ITFunc,(ST_DECR_CNT_MSK|ST_NO_RLD_MSK));
}
//----------------------------------------------------------------------------//
//���������� ����� ������.������� ��������� ������� � ������� �������� ��� NULL � ������ �������.
//���������- ������,��������� �� ������� ��������� ������ ����������,����� ���������
SystemTimer* SetSystemTimer(uint16_t periodMs,void (*ITFunc)(void ),uint16_t flags)
{
  SystemTimer* hti=(pctcon->mode & TMMRAM_STACK_Msk)?mmmalloc(sizeof(SystemTimer)):ccmalloc(sizeof(SystemTimer));
  if(hti==0)return NULL;
  hti->_ty=(uint16_t)TY_SYSTIMER;
  hti->sync=0;
  hti->flags=(uint16_t)(flags|(periodMs?CF_PASSIVE:CF_SIGNAL));
  hti->prev=hti->next=0;
  
  hti->period=periodMs;  //������ ������� � ��
  hti->ITFunc=ITFunc;//��������� ������� ����������
  
  if(_isSysTimerDecr(hti))hti->counter=periodMs;else hti->counter=0; 
  
  //---------� ���������� ?->>>
  if(check_if_IT_less_or_equal())
  {
    if(core_add_CoreObj((PCOREOBJ)hti,0)==(PCOREOBJ)hti)return (SystemTimer*)hti;
  }
  else
  {
    if(call_add_core_obj((PCOREOBJ)hti,pctcon)==(PCOREOBJ)hti)return (SystemTimer*)hti;
  }
  //------------------------<<<
  
  
  mfree(hti);
  return NULL;
}
//----------------------------------------------------------------------------//
//������� �� ������� �������
void svc_isSystemTimerCounterNull(SystemTimer* pTimer)
{
  if(!core_is_system_timer((HANDLE)pTimer) || pTimer->counter==0)
    __out_R0_to_stack_R1_ctx(TRUE,pctcon);
  else __out_R0_to_stack_R1_ctx(FALSE,pctcon);
}
//----------------------------------------------------------------------------//
//��������� ���������� �������.������� �������� ������������ ��� 0 � ������ �������.
void svc_RestartSystemTimer(SystemTimer* pTimer)
{
  if(!core_is_system_timer((HANDLE)pTimer) || (pTimer->period==0))//������ �� �������� ��� �� ��������
  {
    __out_R0_to_stack_R1_ctx(0,pctcon);
    return;
  }
 
  pTimer->flags &=(uint16_t)~(CF_SIGNAL);
  
  if(_isSysTimerDecr(pTimer))pTimer->counter=pTimer->period;//decrementive=reload
  else pTimer->counter=0;//incrementive=0
  
  pTimer->flags &= (uint8_t)~(ST_DIS_INT_MSK|ST_DIS_CNT_MSK);//��������� ��������� ����� � ����
    
  __out_R0_to_stack_R1_ctx(pTimer->period,pctcon);
}
//----------------------------------------------------------------------------//
BOOL DeleteSystemTimer(SystemTimer** ppTimer)//�������
{
  PCOREOBJ pco;
  //-----------� ���������� ?-------->>>
  if(check_if_IT_less_or_equal())
  {
    if(ppTimer==0 || *ppTimer==0 || !core_find_CoreObj((PCOREOBJ)*ppTimer) ||
        ((PCOREOBJ)*ppTimer)->_ty!=TY_SYSTIMER)return FALSE;
    
    pco=core_cut_CoreObj((PCOREOBJ) *ppTimer);//�������� ������ ����
    
    if(pco)
    {
      mfree(pco);
      *ppTimer=0;
      return TRUE;
    }
    else return FALSE;
  }
  else
  {
    return call_DeleteSystemTimer(ppTimer);
  }
  //---------------------------------<<<
}

void svc_DeleteSystemTimer(SystemTimer** ppTimer)
{
  PCOREOBJ pco=0;
  
  if(ppTimer!=0 && *ppTimer!=0 && core_is_system_timer((HANDLE)*ppTimer) )
  {
     pco=core_cut_CoreObj((PCOREOBJ) *ppTimer);//�������� ������ ����
    
    if(pco)
    {
      mfree(pco);
      *ppTimer=0;
    }
  }
  
  __out_R0_to_stack_R1_ctx(pco?TRUE:FALSE,pctcon);
}
//----------------------------------------------------------------------------//
BOOL PauseSystemTimer(SystemTimer* pTimer)
{
  //-----------� ���������� ?-------->>>
  if(check_if_IT_less_or_equal())
  {
    if(pTimer==0 || !core_find_CoreObj((PCOREOBJ) pTimer) ||
        ((PCOREOBJ)pTimer)->_ty!=TY_SYSTIMER)return FALSE;
    
   pTimer->flags|=(uint8_t)(ST_DIS_INT_MSK|ST_DIS_CNT_MSK);//������ ���������� � ����;
   return TRUE;
  }
  else
  {
    return call_PauseSystemTimer(pTimer);
  }
  //---------------------------------<<<
}

void svc_PauseSystemTimer(SystemTimer* pTimer)
{
  if(!core_is_system_timer((HANDLE)pTimer))
  {
    __out_R0_to_stack_R1_ctx(FALSE,pctcon);
    return;
  }
  
  pTimer->flags|=(uint8_t)(ST_DIS_INT_MSK|ST_DIS_CNT_MSK);//������ ���������� � ����;
  
  __out_R0_to_stack_R1_ctx(TRUE,pctcon);
}
//----------------------------------------------------------------------------//
BOOL ResumeSystemTimer(SystemTimer* pTimer)
{
  //-----------� ���������� ?-------->>>
  if(check_if_IT_less_or_equal())
  {
    if(pTimer==0 || !core_find_CoreObj((PCOREOBJ) pTimer) ||
        ((PCOREOBJ)pTimer)->_ty!=TY_SYSTIMER)return FALSE;
    
   pTimer->flags &= (uint8_t)~(ST_DIS_INT_MSK|ST_DIS_CNT_MSK);//��������� ��������� ����� � ����
   return TRUE;
  }
  else
  {
    return call_ResumeSystemTimer(pTimer);
  }
  //---------------------------------<<<
}

void svc_ResumeSystemTimer(SystemTimer* pTimer)
{
  if(!core_is_system_timer((HANDLE)pTimer))
  {
    __out_R0_to_stack_R1_ctx(FALSE,0);
    return;
  }
  
  pTimer->flags &= (uint8_t)~(ST_DIS_INT_MSK|ST_DIS_CNT_MSK);//��������� ��������� ����� � ����
  
  __out_R0_to_stack_R1_ctx(TRUE,pctcon);
}
//----------------------------------------------------------------------------//
void Delay(uint16_t nTimeMs)
{
  SystemTimer* pTimer=SetOnePulseSystemTimer(nTimeMs,NULL);
  if(!pTimer)dbg_error_led(ERR_ZERO_PTR);
  
  WaitForSingleObject((HANDLE)pTimer,INFINITE);
  DeleteSystemTimer(&pTimer);
}

//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//