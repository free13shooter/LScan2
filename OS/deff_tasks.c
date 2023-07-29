/**
  Laser project by dem1305

  @2017
  
  OS deffered tasks
*/
//----------------------------------------------------------------------------//
#include "core.h"
//----------------------------------------------------------------------------//
extern HTHREAD _ts_scan_core_obj();
extern volatile uint32_t tswFLdis0;
extern volatile uint32_t _ccyc_;
extern void _psw_switch_threads_context_to_R0();
void core_scan_tasklist();
//----------------------------------------------------------------------------//
//создание [периодической] задачи [с ограниченным количеством запусков]
void CreateDefferedTask(TFUNC*_pTFunc,void* _pargs,
                        uint16_t flags,
                        uint32_t period,
                        uint32_t numExecutes)
{
  _save_op(25);
  
  Task* pt=mmmalloc(sizeof(Task));
  if(!pt)while(1);
  
  pt->_ty=TY_TASK;
  pt->sync=0;
  pt->pTFunc=_pTFunc;
  pt->pargs=_pargs;
  pt->prev=0;
  pt->flags=flags;
  pt->period=period;
  pt->numExecutes=numExecutes;
  pt->nextRun=_get_timestamp+period;  
  
  uint32_t it=__get_IPSR() & 0xFF;
  //----------------------------------------------------- 
  do
  {
    pt->next=(HANDLE)__LDREXW((unsigned long*)&_taskList_);//(HANDLE)_taskList_;
  }
  while(__STREXW((unsigned long)pt, (unsigned long*)&_taskList_));//_taskList_=pt;
  //----------------------------------------------------- 
  
  if(!it)asm("  svc #33");//scan tasklist
}
//----------------------------------------------------------------------------//
void _core_DeleteTask(Task* ptask)
{
  _save_op(26);
  
  
  if(ptask==0)return;
  
  if(_is_not_core_IT(__get_IPSR() & 0xFF))while (1);//доступ имеет только ядро
 
 //_tasklistlock;
   
  if(ptask->prev)((COREOBJ*)ptask->prev)->next=ptask->next;
  if(ptask->next)((COREOBJ*)ptask->next)->prev=ptask->prev;
  if(ptask->flags & TF_FREE_ARGS)mfree(ptask->pargs);
  
  mfree(ptask);
    
  if(_taskList_==ptask)_taskList_=0;
  
  //_tasklistunlock;
}
//----------------------------------------------------------------------------//
void _core_ExecuteTask(Task* ptask)
{
  _save_op(27);
  
  if(ptask==0)return;
  
  if(ptask->pTFunc)
  {
    CreateThread(ptask->pTFunc,ptask->pargs,/*0*/ptask->flags,0);//В потоке планировщика
  }
  
  if(ptask->numExecutes)
  {
    if(--ptask->numExecutes==0)_core_DeleteTask(ptask);
    return;
  }
  
  if((ptask->flags & TF_PERIODIC)==0)_core_DeleteTask(ptask);
}
//----------------------------------------------------------------------------//
void core_scan_tasklist()
{
  _save_op(28);
  
  Task* pt=(Task*)Atom32Get((unsigned long*)&_taskList_);
  
  for(;pt;)
  {
    if(!pt)break;
    if( ((pt->flags & TF_PERIODIC)==0)||
      ((pt->flags & TF_PERIODIC) && pt->nextRun<=_get_timestamp) )
      {
        if(pt->flags & TF_PERIODIC)pt->nextRun=_get_timestamp+pt->period;
        Task* ptn=(Task*)Atom32Get((unsigned long*)&(pt->next));
        _core_ExecuteTask(pt);
        pt=ptn;
        continue;
      }
    pt=(Task*)Atom32Get((unsigned long*)&(pt->next));
  }
}
//----------------------------------------------------------------------------//
//================= системный таймер+переключения контекстов ===================
//вызывается каждую миллисекунду
/*
void TSV_TIMER_IRQ_HNDLR()
{
  TIM2->SR &= (uint16_t)~TIM_SR_UIF;// Clear the IT update pending Bit
  
  _save_op(29);
  
  SystemTicks++;
  //----цикл по всем объектам ------
  //функции таймеров выполняются с высшим приоритетом.
  HTHREAD pend_thread=_ts_scan_core_obj(); //synс.c
  //-------------------------------------
#if ENABLE_MULTITHREADING==0
  return; //возврат
#endif
  
  core_scan_tasklist();
  
  if(tswFLdis0 & 1)return;//1 = запрет переключения
  
  if(--_ccyc_)return;
  //цикл пройден,смена контекстов
  _ccyc_=TSV_TIMx_CYCLES;
  
  if(tswFLdis0 & 2)
  {
    tswFLdis0 &=(uint32_t) ~2;
    return;
  }
  
  //---------проверка на ожидающий----------
  if(pend_thread) 
  {
    //sub_switch_context_to_R0_(pend_thread);
    //goto _psw_switch_threads_context_to_R0;
   // uint32_t addr=_psw_switch_threads_context_to_R0;
    //goto addr;
  }
  //----------------------------------------
  
  SCB->ICSR |= SCB_ICSR_PENDSVSET_MskVal;
  
}
*/
//----------------------------------------------------------------------------//