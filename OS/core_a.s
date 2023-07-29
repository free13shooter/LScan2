/*==============================================================================
                    core functions+task scheduler
                        @dem1305 15.04.2017
==============================================================================*/
#include "core_def.h"
//----------------------------------------------------------------------------//
        MODULE  task_s
      
        SECTION OS_CORE:CODE(2)
        
//-----------IMPORT---------------------  
        EXTERN  __iar_program_start
        EXTERN  __iar_init_vfp    //FPU init
        EXTERN  __low_level_init
        EXTERN  __iar_data_init3  //glob.data init
        
        /*
        возврат из прерывания с проверкой EX_RETURN 
        и установкой корректного SP
        */
        
        EXTERN sub_retIT_R0_thrd_EX_RETURN 
        
        EXTERN  SystemInit  //RCC
        //EXTERN  HardFault_Handler
        
        EXTERN  __vector_table
        EXTERN  InitSheduler//main sheduler service
        EXTERN  main
        EXTERN  start_svc   //start services
        EXTERN  led_loop
        EXTERN  dbg_error_led
        EXTERN  LSCAN2_LEDToggle
        EXTERN  ccfree //void  ccfree(void* m)//освобождение памяти
   //--------services---------     
   EXTERN  __SVC_add_core_obj          //1 func connect core object
   EXTERN  __SVC_set_not_privileged    //4 func set npriv access
   EXTERN  __SVC_set_privileged        //5 func set priv access
   EXTERN  __SVC_delete_thread         //0
   EXTERN  _psw_switch_threads_context_to_R0
        //C import
        EXTERN  _ts_scan_core_obj
        EXTERN  core_scan_tasklist
//-----------EXPORT FUNCTIONS----------------------  
        PUBLIC  Reset_Handler
        PUBLIC  TSV_TIMER_IRQ_HNDLR
        PUBLIC  UsageFault_Handler
        PUBLIC  BusFault_Handler
        PUBLIC  HardFault_Handler
        PUBLIC  MemManage_Handler
        
        PUBLIC  call_create_new_thread //новый поток
        PUBLIC  call_delete_thread //удаление потока int DeleteThread(TCONTEXT* ptctx);
        PUBLIC  call_get_set_thread_prio //int (HTHREAD pCTX_R0,uint8_t priority_R1,HTHREAD ht_R2,uint8_t bget_set bget_R3);//+
        PUBLIC  ResumeThread      //int ResumeThread(HTHREAD hTrd); //return -1=error
        PUBLIC  SuspendThread     //int SuspendThread(HTHREAD hTrd);//return -1=error
        PUBLIC  call_set_suspend
        PUBLIC  call_set_sleep
        PUBLIC  call_GetThreadId     //int call_GetThreadId(HTHREAD hTrd);//return -1=error
        PUBLIC  call_GetThreadHandle //HTHREAD call_GetThreadHandle(int id);//return -1=error
        PUBLIC  ExitThread //int ExitThread(int)завершение текущего потока сзаданным кодом возврата
        
        PUBLIC  call_add_core_obj
        PUBLIC  call_save_thread_sync_parameters
        PUBLIC  call_rls_object
        PUBLIC  call_change_core_object_flags
        PUBLIC  call_CloseHandle
        PUBLIC  call_SW_thread  //переключить поток
        PUBLIC  call_find_handle
        
        PUBLIC  tfunc_wrapper //обертка функции потока
        PUBLIC  twrap_del     //удаление функции потока
//-----------EXPORT VARIABLES----------------------        
        PUBLIC  pctcon    //текущий контекст
        PUBLIC  _proot_   //anchor of list,root coreobj* 
        PUBLIC  _pend_    //end of list
        PUBLIC  _otot_    //total objects
        PUBLIC   _ccyc_
                
        PUBLIC  SystemTicks //sys timer inc.val
        PUBLIC  tswFLdis0   //SW flags
        
#if CORE_DBG_OPERATION==1
        PUBLIC _dbg_op //(CORE_DBG_LEN) байт
        PUBLIC _dbi    //индекс
#endif
        //===ФУНКЦИИ СИСТЕМНОГО ТАЙМЕРА===
        PUBLIC  call_RestartSystemTimer 
        PUBLIC  call_DeleteSystemTimer
        PUBLIC  call_PauseSystemTimer
        PUBLIC  call_ResumeSystemTimer
        PUBLIC  call_IsSystemTimerCounterNull
        PUBLIC  call_IsSystemTimer
        //==Флаги блокировок====
        PUBLIC  _mm_flag_lock_     //volatile unsigned char,memory manager lock flag 
        PUBLIC _core_flag_lock     //volatile unsigned char _core_flag_lock;
        //PUBLIC _taskList_flag_lock //флаг блокировки списка задач для синхронизации доступа
        //==список задач для выполнения==
        PUBLIC _taskList_      //volatile Task* _taskList_ 
//==============================================================================
        SECTION CSTACK:DATA:NOROOT(3)
        
        SECTION OS_CORE_DATA:DATA:ROOT(4)    //ALIGN 4
//------
_proot_         DCD   0   //якорь списка объектов ядра
_pend_          DCD   0   //конец списка объектов ядра
_otot_          DCD   0   //количество объектов ядра

pctcon          DCD   0   //текущий поток 

_taskList_      DCD   0   //список задач для обработки , volatile Task* _taskList_ 
//------------------------------------------------------------------------------  
SystemTicks     DCD   0   //счетчик импульсов таймера переключения контекстов (1 мс)
//------------------------------------------------------------------------------  
tswFLdis0       DCD   0 //0   //флаги переключения
/* биты :
[0] == 1-запрет переключения 
[1] == 2-пропуск одного переключения
*/
_ccyc_          DCB  TSV_TIMx_CYCLES   // циклов до переключения
//---------------------------------------------------------
_mm_flag_lock_      DCB   0 //флаг блокировки менеджера памяти
/*
флаг блокирования для выявления доступа более приоритетного прерывания
(вызовет зависание,т.к. доступ имеет только ядро)
*/
_core_flag_lock     DCB  0 //static volatile unsigned char _core_flag_lock=0;

//флаг блокировки списка задач для синхронизации доступа
//_taskList_flag_lock DCB  0 //static volatile unsigned char _core_flag_lock=0;
 //---------------------------------------------------------
 #if CORE_DBG_OPERATION==1
                DS8 11     //
_dbg_op         DS8 (CORE_DBG_LEN)     //4 bytes array  
_dbi            DCB 0
#endif
//---------------------------------------------------------
        THUMB
//=========================сброс,инициализация==================================
        SECTION OS_CORE:CODE(2)
 #if CORE_DBG_OPERATION==1
//последняя операция ядра. Применяется для отладки.
_save_op  MACRO _numop_
          PUSH {R0-R3}
          LDR.W R1,=_dbg_op
          LDR.W R3,=_dbi
          LDR R2,[R3]
          MOV R0,#(_numop_) 
          STRB R0,[R1,R2]
          ADD R2,R2,#1
          CMP R2,#(CORE_DBG_LEN)
          IT EQ
          MOVEQ R2,#0
          STRB R2,[R3]
          POP {R0-R3}
          ENDM
#else
_save_op  MACRO _numop_
          ENDM
#endif       


Reset_Handler
        LDR.W   R0, =SystemInit
        BLX     R0
   
        //LDR     R0, =init_sheduler
        //LDR     R0, =__iar_program_start
        //BX      R0
        
init_sheduler:           
 /*
 __iar_program_start инициализация глобальных переменных и библиотек
 инициализация констант ассемблера в __low_lewel_init
 инициализация глобальных структур C в __iar_data_init3
*/     
        //инициализация массивов и переменных
        BL  __low_level_init
        CMP R0,#0
        BEQ _init_root_
        BL  __iar_data_init3 //инициализация глобальных переменных и массивов
        //-----выравнивание стека------
        //LDR R2, =SBC_CCR_REG //=0xE000ED14 
        //LDR R0,[R2]
        //BIC R0,R0,#0x200 //0x200 = 8b align else 4b align
        //STR R0,[R2]
        //LDR R0,[R2]
        //AND R0,R0,#(STKALIGN_VAL) 
        //-----
//================= запуск корневого потока (+планировщик)======================          
/*
1)используйте инструкцию MSR CONTROL,чтобы установить бит активного указателя стека в 1,
2) выполнить возврат исключения в режим Thread с соответствующим EXC_RETURN
-При изменении указателя стека программное обеспечение должно использовать инструкцию ISB сразу после
инструкции MSR. Это гарантирует, что инструкции после ISB используют новый указатель стека
*/
_init_root_:  
//CONTROL:4=use FPU,2=use PSP,1=unprivileged mode

#if ENABLE_USE_FPU==1
        BL __iar_init_vfp  //FPU init          
#endif          
        //B main
        BL InitSheduler //init sys timer and root thread
        
        
        LDR R1,=_proot_
        LDR R1,[R1]
        LDR R2,=pctcon
        STR R1,[R2]
        LDR R3,=_pend_
        STR R1,[R3]

        
		//-------------------
		//подготовка стека и режима доступа
		LDR R0,[R1,#offs_mode]
        //PRIV? (CONTROL 0=PRIV/1=NO_PRIV)
        MRS R3,CONTROL
        AND R3,R3,#(~7) //1=NPRIV 2=PSP 4=FPU
        ANDS R2,R0,#TM_NPRIV
        IT NE
          ORRNE R3,R3,#1 //NPRIV
        ANDS R2,R0,#TM_PSP
        IT NE
          ORRNE R3,R3,#2

#if ENABLE_USE_FPU
        ANDS R2,R0,#TM_FPU
        IT NE
          ORRNE R3,R3,#4
#endif

        LDR R0,[R1,#offs_tstack_PSP]
        MSR PSP,R0//установить стек

        MSR CONTROL,R3 //1=NPRIV 2=PSP 4=FPU
		    ISB           //сброс конвеера инструкций
        
#if ENABLE_MULTITHREADING==1
		    BL start_svc
#endif                
        BL main
        
        B twrap_del
//========================обертка функции потока================================ 
//выполнение функции потока и зачистка по завершению,возврат результата в R0
//R1=func R0=arg R2=ctx
tfunc_wrapper:
   BLX R1 
   //R0==значение возврата потока
twrap_del: 
   LDR R2,=pctcon //текущий
   MOV R4,R0
   LDR R0,[R2] //R0=удаляемый контекст
   //---------
   SVC #0//R4==удаляемый контекст
   
   MOV R0,R4 //ret val
   //удаленный поток выйдет из цикла при переключении контекстов
twrap_ret:
   //WFI
   B twrap_ret//led_loop
//================= системный таймер+переключения контекстов ===================
//вызывается каждую миллисекунду

TSV_TIMER_IRQ_HNDLR:

  PUSH {R0-R2,LR}
  //TIM2->SR &= (uint16_t)~TIM_SR_UIF;// Clear the IT update pending Bit
  LDR.N     R2, =(TSV_TIMER_BASE_ADR+TIMx_SR_OFFSET)
  LDRH      R0, [R2]
  MOVW      R1, #0xfffe
  ANDS      R0, R0, R1
  STRH      R0, [R2]
  
  LDR R1,=SystemTicks //++
  LDR R0,[R1]
  ADD R0,R0,#1
  STR R0,[R1]
  //----цикл по всем объектам ------
  //функции таймеров выполняются с высшим приоритетом.
  BL _ts_scan_core_obj //synс.c
  //-------------------------------------
#if ENABLE_MULTITHREADING==0
  POP {R0-R2,PC} //возврат
#endif
  
  PUSH {R0} //PEND ?
  
  BL core_scan_tasklist
  
  POP {R3}
  
ttt_chdis0:
  LDR R0,=tswFLdis0 //флаги ядра
  LDR R1,[R0]
  AND R1,R1,#1
  CMP R1,#1
  BEQ.N ttt_ret //1 = запрет переключения
  //---------проверка на ожидающий----------
  CMP R3,#0
  BEQ.N ttt_no_pend
  //ITTT NE
    POP {R0-R2,LR} 
    MOV R0,R3
    B _psw_switch_threads_context_to_R0
  //----------------------------------------
ttt_no_pend:  
  LDR   R0,=_ccyc_
  LDRB   R1,[R0]
  SUB  R1,R1,#1
  CMP   R1,#0
  ITT   NE
    STRBNE R1,[R0]
    BNE.N ttt_ret //цикл не пройден 
  //цикл пройден,смена контекстов
  MOV  R1,#(TSV_TIMx_CYCLES)
  STRB R1,[R0]
  
  //требуется ли пропустить переключение контекстов ?
  LDR R0,=tswFLdis0
  LDR R1,[R0]
  AND R2,R1,#2 //пропуск?
  AND R1,R1,#0xFFFFFFFD //сброс пропуска
  STR R1,[R0]
  CBNZ R2,ttt_ret //2 = требуется пропуск
  
  LDR R0,=pctcon //текущий
  LDR R0,[R0]
  LDR.W R1,[R0,#offs_mode] 
  AND.W R1,R1,#(TCRITSECT_Msk)
  CBNZ R1,ttt_ret //PC в критической секции
  //---переключение контекста-------------
  LDR R0,=(SCB_BASE+SCB_ICSR_OFFSET)
  LDR R1,[R0]
  ORR R1,R1,#(SCB_ICSR_PENDSVSET_MskVal) //установить флаг прерывания PendSV
  STR R1,[R0]
  //--------------------------------------
ttt_ret:
  
  POP {R0-R2,PC}

//========================функция удаления потока===============================
call_delete_thread:
    PUSH {LR}
    SVC #0 //R0==удаляемый контекст
    POP {PC}
//=============функция изменения приоритета потока==============================
//int call_get_set_thread_prio(HTHREAD pCTX_R0,uint8_t priority_R1,HTHREAD ht_R2,uint8_t bget_set_R3);//+
call_get_set_thread_prio:
    PUSH {LR}
    SVC #3 //R0=pctx R1=prio R2=stack out ctx, R3=0==get/1==set
    POP {PC}
//=================== приостановить поток R0 на R1 ms ==========================  
//если R0==0-загрузить в R0 текущий поток
call_set_sleep:
  PUSH {LR}
  _save_op 30 
  SVC #6
  POP {PC}
//================= установки маски suspend R1 потока R0 ======================= 
//если R0==0-загрузить в R0 текущий поток
ResumeThread:
  MOV R1,#0
  _save_op 31 
  B call_set_suspend
SuspendThread:
  _save_op 32 
  MOV R1,#(TM_CREATE_SUSPENDED)
call_set_suspend:
  PUSH {LR}
  SVC #7
  POP {PC}
//================= вернуть ID потока R0 ======================================= 
//если R0==0-загрузить в R0 HTHREAD текущего потока (поиск по HTHREAD)
call_GetThreadId:
  PUSH {LR}
  SVC #10
  POP {PC}
//================= вернуть HTHREAD потока с ID==R0 ============================
//если R0==0-загрузить в R0 ID текущeго потока (поиск по ID)
call_GetThreadHandle:
  PUSH {LR}
  SVC #11
  POP {PC}
//================= подключить объект к списку =================================
//R0=объект для подключения
call_add_core_obj:
  PUSH {LR}
  _save_op 33 
  SVC #1
  POP {PC}
//=== завершить текущий поток с заданным кодом возврата (R0)====================
ExitThread:
  BL twrap_del
//===========запись параметров синхронизации в контекст потока==================   
call_save_thread_sync_parameters:
  PUSH {LR}
  _save_op 34 
  SVC #19
  POP {PC}
//=========================освободить объект====================================   
//BOOL call_rls_object(HANDLE pco_R0,HTHREAD ht_R1)
call_rls_object:
  PUSH {LR}
  _save_op 35 
  SVC #20
  POP {PC}
//====================изменить флаги объекта ядра===============================
//R0=объект ядра R1=(uint16_t)flags R2=SET/RESET
call_change_core_object_flags:  
  PUSH {LR}
  _save_op 36 
  SVC #21
  POP {PC}
//====================освободить ресурсы дескриптора============================  
call_CloseHandle: 
  PUSH {LR}
  _save_op 37 
  SVC #22
  POP {PC}
//===============принудительное переключение контекста========================== 
call_SW_thread: 
  PUSH {LR}
  SVC #2
  POP {PC}
//======================поиск объекта=========================================== 
call_find_handle: 
  PUSH {LR}
  _save_op 38 
  SVC #23
  POP {PC}
//======================ФУНКЦИИ СИСТЕМНОГО ТАЙМЕРА==============================
call_RestartSystemTimer: 
  PUSH {LR}
  _save_op 39 
  SVC #24 
  POP {PC}
  
call_DeleteSystemTimer:
  PUSH {LR}
  _save_op 40 
  SVC #25
  POP {PC}
  
call_PauseSystemTimer:
  PUSH {LR}
  _save_op 41 
  SVC #26
  POP {PC}
call_ResumeSystemTimer:
  PUSH {LR}
  _save_op 42 
  SVC #27
  POP {PC}
call_IsSystemTimerCounterNull:
  PUSH {LR}
  _save_op 43 
  SVC #28
  POP {PC}
call_IsSystemTimer:
  PUSH {LR}
  _save_op 44 
  SVC #29
  POP {PC}
//=============================================================================
/*
создание нового потока (процесса)  
R0==param {+0:(uint32_t)pFunc,+4:(uint32_t)pArgs,+8:(uint32_t)pCTX}; (в стеке)
        //[17FPU regs] PSR PC LR R12 R3 R2 R1 R0       
*/
call_create_new_thread:
  PUSH {LR}
  _save_op 45 
  SVC #31
  POP {PC}
//=============================================================================  
HardFault_Handler:
  B.N UsageFault_Handler//HardFault_Handler //UsageFault_Handler
HF_LOOP:
  B.N HF_LOOP
/*
BusFault: обнаруживает ошибки доступа к памяти при извлечении команды,
считывание / запись данных, выборка прерывания,
и записывать стекирование (сохранение / восстановление)
при прерывании (ввод / вывод).  
*/
BusFault_Handler:
  //B.N _check_current_IT
  B.N UsageFault_Handler //UsageFault_Handler
//BF_LOOP:
  //B.N BF_LOOP

MemManage_Handler:
  B.N UsageFault_Handler //UsageFault_Handler

UsageFault_Handler:
  
  TST   LR, #4        //EXC_RETURN (bit 2)?
  ITE   EQ            //if Z - MSP else PSP
  MRSEQ R12, MSP       //MSP->R0
  MRSNE R12, PSP       //PSP->R0 (bit set)
  //извлечь параметры: в стеке PSR PC LR R12 R3 R2 R1 R0
  LDM R12, {R0-R3}
  LDR R12, [R12,#24]    //извлечь PC из стека
  
  LDR R4, =pctcon           //указатель на контекст
  LDR R4,[R4]               //значение указателя
  LDR R5,[R4,#offs_tstaRAM_PSP] //низ стека
  LDR R6,[R4,#offs_tstsize_PSP] 
  LDR R7,[R4,#offs_tstack_PSP]
  LDR R8,[R4,#offs_mode]
  
  LDR R9, [R4,#offs_id]
  
UF_LOOP:
  B.N UF_LOOP
  //BX LR
//=============================================================================  
_check_current_IT:
   // Загрузите адрес регистра управления прерываниями в r3.  
   LDR.W  R3,=(NVIC_INT_CTRL_CONST) //LDR.W R3, #(0xE000ED04) //NVIC_INT_CTRL_CONST
// Загрузите значение регистра управления прерываниями в r2 из 
//адреса, хранящегося в r3.  * /
   ldr r2, [r3, # 0]
// Номер прерывания находится в младшем значении байта - очистить все 
//другие биты. 
   uxtb r2, r2
Infinite_Loop:
   // Теперь сидите в бесконечном цикле - номер исполняемого прерывания 
   // проводится в r2.  
   b Infinite_Loop
   //.size Default_Handler,.-Default_Handler

 //.align 4
 // Адрес регистра управления прерываниями NVIC.  
 //NVIC_INT_CTRL_CONST: .word 0xe000ed04
//============================================================================= 
  END
