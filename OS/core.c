/**
  Laser project by dem1305

  @2017
  
  OS core
*/
//----------------------------------------------------------------------------//
#include "core.h"
#include "Audio.h"
//----------------------------------------------------------------------------//
/*
резервировние памяти для стека/стеков .
стек приложения настраиваемый,если указано, иначе размер по умолчанию.
стек может быть помещен в CCRAM.
*/
void create_stacks(TCONTEXT* pt,uint32_t stack_size)
{
    _save_op(1);
    pt->tstsize_PSP=(uint32_t)DEFAULT_TSTACK_SIZE;
    if((pt->mode & TSTACKSIZE_Msk) && stack_size!=0)pt->tstsize_PSP=stack_size;
    pt->tstaRAM_PSP=(uint32_t)((pt->mode & TMMRAM_STACK_Msk)?mmmalloc(pt->tstsize_PSP):ccmalloc(pt->tstsize_PSP));
    if(pt->tstaRAM_PSP==0)while(1);
    pt->tstack_PSP=pt->tstaRAM_PSP+pt->tstsize_PSP-1;
    pt->tstack_PSP=_aligndown_(pt->tstack_PSP,8);//выравнивание 8 байт (4 слова)
}
//----------------------------------------------------------------------------//
void  InitSheduler()
{
  _save_op(2);
  /*
  Функция NVIC_SetPriorityGrouping(priority_grouping) записывает значение своего аргумента в поле 
  PRIGROUP регистра SCB_AIRCR. Это сообщает NVIC, сколько битов приоритета является приоритетом группы и подприоритетом.
  */
  //вроде делает то же, что и NVIC_PriorityGroupConfig. Настоящее уёбство в понимании.
  //AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
  //SCB->AIRCR=((uint32_t)0x5FA << 16) | (2 << 8)); =>  SCB->AIRCR=((uint32_t)0x5FA0000) | (0x200)) НЕ СООТВЕТСТВУЕТ 0x05FA0000|0x500 NVIC_PriorityGroupConfig
  //NVIC_SetPriorityGrouping(5);//0x101==[0-3]:[0-3] !!! ЭТО ТОЛЬКО ЗНАЧЕНИЕ ПОЛЯ PRIGROUP, требуется сдвиг <<8
  //uint32_t gr=NVIC_GetPriorityGrouping();
   
  //задать способ разделения приоритетов и подприоритетов прерываний 
  //SCB->AIRCR=0x05FA0000|0x500
  NVIC_PriorityGroupConfig(CORE_NVIC_PriorityGroup);//2==[0-3]:[0-3] - ПРАВИЛЬНО ПО ТАБЛИЦЕ 51 PM0214
  uint32_t gr=NVIC_GetPriorityGrouping();
  //Bits 31:24 PRI_11: Priority of system handler 11, SVCall
  NVIC_set_group_prio(SVCall_IRQn,TSVCALL_IT_PRIO,TSVCALL_IT_PRIO_SUB);
  NVIC_set_group_prio(PendSV_IRQn,TPENDSV_IT_PRIO,TPENDSV_IT_PRIO_SUB);
  //NVIC_SetPriority(SVCall_IRQn, (uint32_t)((TSVCALL_IT_PRIO<<prio_pt_pos(CORE_NVIC_PriorityGroup))|TSVCALL_IT_PRIO_SUB));
  //NVIC_SetPriority(PendSV_IRQn, (uint32_t)(TPENDSV_IT_PRIO|TPENDSV_IT_PRIO_SUB));
  
  SCB->SHCSR|=(uint32_t)(SCB_SHCSR_MEMFAULTENA_Msk|SCB_SHCSR_BUSFAULTENA_Msk|SCB_SHCSR_USGFAULTENA_Msk);
  
  SCB->CCR|=0x200;
  
#if DISABLE_MCU_BUFFERING_MODE==1
  SCnSCB->ACTLR|=SCnSCB_ACTLR_DISDEFWBUF_Msk;//выкл буферизации(эскалация BUS- сбоев для определения)
#endif
  //----------
  initheaps();//инициализировать кучу CCRAM/RAM
  
  //создание контекста потока
  //#pragma data_alignment = 4

  TCONTEXT* pCTX=(TCONTEXT*)ccmalloc(sizeof(TCONTEXT));
  
  if(!pCTX)while(1);//ERROR !!!
  
  ZeroMemory(pCTX,sizeof(TCONTEXT));
  
  pCTX->id=(uint32_t)1;
  
  pCTX->mode=TM_PRIV|TM_PSP |(((uint32_t)DEFAULT_TPRIORITY)<<TPRIO_BIT_Pos);   //npriv+stack+prio lvl modes

#if ENABLE_USE_FPU==1
  pCTX->mode|=((uint32_t)TM_FPU);   //need init FPU
#else
  
#endif

  create_stacks(pCTX,0);
  
  _proot_=(PCOREOBJ)pCTX;
  
#if THREAD_CONTEXT_MPU_PROTECTION==1
  MPU_set_region(0,(void*)pCTX,MPU_SIZE_128,AT_RWX,PRIVDEFENA|MPUENABLE,true); //-------ДОРАБОТАТЬ
#endif
  //---------
  //-system timer-svcall ---
  //PCLK2=84 MHZ,тактирование таймера 168 мгц.
  //PCLK1=42 MHZ,тактирование таймера 84 мгц.
  TSV_TIMER_RCC_APBCmd(TSV_TIMER_APBPeriph, ENABLE);
  TSV_TIMER->PSC = TSV_TIMER_PSC_1MS*(uint16_t)TSV_TIME_MS-(uint16_t)1;
  TSV_TIMER->ARR = TSV_TIMER_ARR_1MS;
  TSV_TIMER->CNT = 1;//счет вверх
  TSV_TIMER->EGR = TIM_EGR_UG; //TIM event generation register
  TSV_TIMER->SR &= (uint16_t)~TIM_SR_UIF;// Clear the IT pending Bit
} 
//----------------------------------------------------------------------------//
void sw_now_thread_to(HTHREAD pt)
{
  _save_op(3);
  __asm("svc #32");
}
//----------------------------------------------------------------------------//
void start_svc()
{
  _save_op(4);
  
  NVIC_set_group_prio(TSV_TIMER_IRQ,TTIMER_IT_PRIO,TTIMER_IT_PRIO_SUB);
  //------------------------
  NVIC_EnableIRQ(TSV_TIMER_IRQ);
  TSV_TIMER->CR1 = TIM_CR1_URS|TIM_CR1_CEN;//режим+запуск,обработка в прерываниях
  TSV_TIMER->DIER = TIM_DIER_UIE; //Разрешаем прерывание при переполнении счетчика
  
  tswFLdis0=(uint32_t)2;//запрет одного переключения
  //sw_now_thread_to((HTHREAD)_proot_);
  
}
//----------------------------------------------------------------------------//
void led_loop() {
  //RBGO
  uint32_t p=2000000;
 
  while (1) {
    LSCAN2_LEDOn(LEDBLUE);LSCAN2_LEDOff(LEDRED);
    for(int i=p;i>0;i--){;}
    LSCAN2_LEDOn(LEDGREEN);LSCAN2_LEDOff(LEDBLUE);
    for(int i=p;i>0;i--){;}
    LSCAN2_LEDOn(LEDWHITE);LSCAN2_LEDOff( LEDGREEN);
    for(int i=p;i>0;i--){;}
    LSCAN2_LEDOn(LEDRED);LSCAN2_LEDOff( LEDWHITE);
    for(int i=p;i>0;i--){;}
  }
}
//----------------------------------------------------------------------------//
//создание потока,возврат указателя на контекст CTX
HANDLE CreateThread(TFUNC* pFunc,void* pArgs,uint32_t mode_flags,uint32_t stack_size)
{
  _save_op(5);
  uint32_t it=check_if_IT_less_or_equal();
  //создание контекста потока
  //#pragma data_alignment = 4
  TCONTEXT* pCTX=( (TCONTEXT*)((mode_flags & TMMRAM_CTX_Msk)?mmmalloc(sizeof(TCONTEXT)):ccmalloc(sizeof(TCONTEXT))) );

  if(!pCTX)return 0;
  
  ZeroMemory(pCTX,sizeof(TCONTEXT));
  
  pCTX->mode=mode_flags & (~(uint32_t)TM_FPU);
  
  if(mode_flags==0)
  {
    pCTX->mode=TM_NPRIV|TM_PSP |(((uint32_t)DEFAULT_TPRIORITY)<<TPRIO_BIT_Pos);   //npriv+stack+prio lvl modes
    
#if ENABLE_USE_FPU==1
    pCTX->mode|=((uint32_t)TM_FPU);   //need store FPU registers
#endif
    
  }
  else
  {
  
    if(mode_flags & TM_FPU)pCTX->mode|=((uint32_t)TM_FPU);   //need store FPU registers

    pCTX->mode|=(uint32_t)TM_PSP;
  }
  
  create_stacks(pCTX,stack_size);
  
  if(!it)EnterCriticalSection;
  //-------scan for minimal id num-------
  TCONTEXT* t=(TCONTEXT*)_proot_;
  for(;t!=0;t=(TCONTEXT*)t->next)
  {
    if(t->id-pCTX->id>1){pCTX->id++;break;}
    else pCTX->id=t->id;
  }
  
  if(t==0)pCTX->id++;//все по порядку
  //-------------------------------------
  if(!it)LeaveCriticalSection;
  
  //t=(TCONTEXT*)_proot_;
  
  uint32_t args[3]={(uint32_t)pFunc,(uint32_t)pArgs,(uint32_t)pCTX};
  
  if(!it)
  {
    return (HANDLE)call_create_new_thread((uint32_t)&args,CurrActiveThreadCTX);
  }
  else
  {
    return (HANDLE)svc_new_thread((uint32_t)&args,0);
  }
}
//----------------------------------------------------------------------------//
//бесконечный цикл для фиксирования ошибок при отладке
void dbg_error_led(uint32_t errormask_R0)
{
  
  uint8_t ofs=0;
  SoundGain=0.1f;
  disableAudio=true;
  
  while (1)
  {
    Beep(2000,300);//error
    if(ofs>=7)ofs=0;
    if(errormask_R0&(1UL<<ofs)) LSCAN2_LEDOn(LEDRED);
    if(errormask_R0&(2UL<<ofs)) LSCAN2_LEDOn(LEDGREEN);
    if(errormask_R0&(4UL<<ofs)) LSCAN2_LEDOn(LEDBLUE);
    if(errormask_R0&(8UL<<ofs)) LSCAN2_LEDOn(LEDORANGE);
    it_mDelay(250);
    Beep(200,300);//error
    ofs+=4;
    if((errormask_R0&(1UL<<ofs))==0) LSCAN2_LEDOff(LEDRED);
    if((errormask_R0&(2UL<<ofs))==0) LSCAN2_LEDOff(LEDGREEN);
    if((errormask_R0&(4UL<<ofs))==0) LSCAN2_LEDOff(LEDBLUE);
    if((errormask_R0&(8UL<<ofs))==0) LSCAN2_LEDOff(LEDORANGE);
    it_mDelay(250);
    ofs+=4;
  }
}
//----------------------------------------------------------------------------//
// #22
//закрыть дескриптор.Если  ht_R1 не указан,вывода в стек не будет.   
BOOL svc_close_hndl(HANDLE pco,HANDLE ht_R1)
{
  _save_op(6);
  
  Mutex* pmu;Semaphore* ps;TCONTEXT* ht;TCONTEXT* ht1;uint32_t cnt=0;
  
  uint32_t it=check_if_IT_less_or_equal();
  
  if(pco==NULL || (core_find_CoreObj((PCOREOBJ)pco)==FALSE) || ((PCOREOBJ)pco)->_ty>TY_MAX)
    goto _svcclose_ret_FALSE;
  
  core_cut_CoreObj_no_find((PCOREOBJ)pco);
  //---------------
  switch(((PCOREOBJ)pco)->_ty)
  {
    case TY_MUTEX:
      pmu=(Mutex*)pco;
      if(pmu->sync!=(HANDLE*)CurrentActiveThreadCTX && (pmu->sync!=0) && ((pmu->flags & CF_SIGNAL)==0))//not owner==error
       goto _svcclose_ret_FALSE;
      break;
    //----------
    case TY_SEMAPHORE:  
      ps=(Semaphore*)pco;
      
      for(uint32_t i=0;i<ps->maxcount;i++)
      {
        if(ps->sync[i])cnt++;
        if(ps->sync[i]==(HANDLE)CurrentActiveThreadCTX && ps->count<ps->maxcount)
        {
          ps->sync[i]=0;ps->count++;cnt--;
        }
      }
      if(cnt>0)goto _svcclose_ret_FALSE;//not found==error
      mfree(ps->sync);//освободить память массива
      break;
    //----------
    case TY_THREAD: 
      ht=(TCONTEXT*)pco;
      
      if(ht==pctcon)
      {
        ht1=select_next_SW_thread();
        if(ht1)sub_switch_context_to_R0_(ht);
      }
      
      
     if(ht->mode & TM_PSP)mfree((void*)ht->tstaRAM_PSP); //освободить память стека
     if(ht!=pctcon && ht_R1)__out_R0_to_stack_R1_ctx(TRUE,(HTHREAD)ht_R1);//не текущий
     mfree(pco);//освободить память объекта
     return TRUE;
    //----------
    case TY_EVENT: 
      break;
    //----------
    case TY_SYSTIMER: 
      break;
    //----------
    default:
      goto _svcclose_ret_FALSE;//error
      break;
   }  
   //------------------------
  mfree(pco);//освободить память объекта
  if(ht_R1)__out_R0_to_stack_R1_ctx(TRUE,(HTHREAD)ht_R1);
  return TRUE;
  
_svcclose_ret_FALSE:  
  if(ht_R1)__out_R0_to_stack_R1_ctx(FALSE,(HTHREAD)ht_R1);
  return FALSE;
}
//----------------------------------------------------------------------------//
bool core_find_CoreObj(PCOREOBJ pobj)//найти объект ядра
{
  _save_op(7);
  
  for(PCOREOBJ pco=_proot_;pco!=0;pco=(PCOREOBJ)pco->next)
  {
    if(pco==pobj)return true;//найден
  }
  return false;
}
//----------------------------------------------------------------------------//
//R0==вырезаемый объект.Вернет вырезанный контекст при успехе или 0==ошибка

//без проверки (поиска)
PCOREOBJ core_cut_CoreObj_no_find(PCOREOBJ pobj)//вырезать объект ядра
{
  _save_op(8);
  
  uint32_t it=check_if_IT_less_or_equal();
  
  _corelock;
  
  PCOREOBJ pprev=(PCOREOBJ)pobj->prev;
  PCOREOBJ pnext=(PCOREOBJ)pobj->next;
  
  if(pprev)pprev->next=(HANDLE)pnext;
  if(pnext)pnext->prev=(HANDLE)pprev;
  
  if(_proot_==pobj)_proot_=pnext;
  if(_pend_==pobj)_pend_=pprev;
  
  _otot_--;
  
  __asm("DSB");
  
  _coreunlock;
  
  return pobj;
}

PCOREOBJ core_cut_CoreObj(PCOREOBJ pobj)//вырезать объект ядра
{
  _save_op(9);
  
  if(!core_find_CoreObj((PCOREOBJ)pobj))
  {
   return 0; 
  }
  
  return core_cut_CoreObj_no_find(pobj);
}
//----------------------------------------------------------------------------//
//добавить объект к списку.Если поток не указан, вывода в стек не будет.
PCOREOBJ core_add_CoreObj(PCOREOBJ pobj,HANDLE ht_R1)//добавить в список ядра объект ядра
{
  _save_op(10);
  
  uint32_t it=check_if_IT_less_or_equal();
  //R0==подключаемый объект
  _corelock;
  
  if(_proot_==0)_proot_=pobj;
  _pend_->next=(HANDLE)pobj;
  pobj->prev=(HANDLE)_pend_;
  pobj->next=0;
  _pend_=pobj;
  ++_otot_;
  
  if(ht_R1)__out_R0_to_stack_R1_ctx((uint32_t)pobj,(HTHREAD)ht_R1);
  _coreunlock;
  return pobj;
}
//----------------------------------------------------------------------------//
//изменить флаги объекта.Если поток не указан, вывода в стек не будет.
HANDLE core_change_core_object_flags(PCOREOBJ pco_R0,uint16_t flags_R1,uint8_t SET_RESET_R2,HANDLE ht_R3)
{
  _save_op(11);
  
  uint32_t it=check_if_IT_less_or_equal();
  
  HANDLE res=0;
  
  if(core_find_CoreObj(pco_R0))
  {
    if(SET_RESET_R2==SET)
    {
      pco_R0->flags|=flags_R1;
    }
    else
    {
      pco_R0->flags&=~flags_R1;
    }
    
    res=(HANDLE)pco_R0;
  }
  
  if(ht_R3)__out_R0_to_stack_R1_ctx((uint32_t)pco_R0,(HTHREAD)ht_R3);
  return res;
}
//----------------------------------------------------------------------------//
void core_debug(TCONTEXT* ht)
{
  TCONTEXT* ct=ht;
  while(1);
}
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
BOOL CloseHandle(HANDLE pco)//освободить ресурсы дескриптора объекта
{
  _save_op(12);
  
  if(check_if_IT_less_or_equal())
  {
    return svc_close_hndl(pco,0);
  }
  else
  {
    return call_CloseHandle(pco,CurrActiveThreadCTX);
  }
}
//----------------------------------------------------------------------------//
//set/get prio.level
int SetThreadPriorityLevel(HTHREAD pCTX,uint8_t priority)//set priority
{
  if(check_if_IT_less_or_equal())
  {
    return svc_get_set_thread_prio(pCTX,priority,0,1);
  }
  
  return call_get_set_thread_prio(pCTX,priority,CurrActiveThreadCTX,1);
}

int GetThreadPriorityLevel(HTHREAD pCTX) 
{
  if(check_if_IT_less_or_equal())
  {
    return svc_get_set_thread_prio(pCTX,0,0,0);
  }
  
  return call_get_set_thread_prio(pCTX,0,CurrActiveThreadCTX,0);
}
//----------------------------------------------------------------------------//
int GetThreadId(HTHREAD hTrd)//return -1=error
{
  HTHREAD ht=hTrd==0?CurrActiveThreadCTX:hTrd;
  
  if(check_if_IT_less_or_equal())
  {
    return svc_get_thread_id(ht,0);
  }
  
  return call_GetThreadId(ht,CurrActiveThreadCTX);
}

HANDLE GetThreadHandle(int id)//return -1=error
{
  if(check_if_IT_less_or_equal())
  {  
    return svc_get_thread_handle(id,0);
  }
  
  return call_GetThreadHandle(id,CurrActiveThreadCTX);
}
//----------------------------------------------------------------------------//
uint32_t check_if_IT_less_or_equal()
{
  uint32_t it=(__get_IPSR() & 0xFF);//недопустимо в прерывании
  if(it)
  {
    if(_is_not_core_IT(it))//while(1);
    {
      if(_is_prio_Lvl_more_Core(it))while(1);
      /*
      ЗАПРЕЩЕН ДОСТУП ПРЕРЫВАНИЙ С ПРИОРИТЕТОМ ВЫШЕ ПРЕРЫВАНИЙ ЯДРА-
      ЭТО ВЫЗОВЕТ СБОЙ ДОСТУПА К ДАННЫМ ЯДРА И РАЗРУШЕНИЕ СПИСКОВ ОБЪЕКТОВ ЯДРА
      */
    }
  }
  return it;
}
//----------------------------------------------------------------------------//
