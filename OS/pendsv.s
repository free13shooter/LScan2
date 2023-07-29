/*==============================================================================
                            pendsv
                        @dem1305 15.04.2017
==============================================================================*/
#include "core_def.h"

//System handler priority register 2 (SHPR2) ==SVcall prio.


//----------------------------------------------------------------------------//
        MODULE  PENDSVC
        SECTION OS_CORE:CODE(2)  //align 8 bytes
        
#if CORE_DBG_OPERATION==1
        EXTERN _dbg_op //(CORE_DBG_LEN) ����
        EXTERN _dbi
#endif
//-----------IMPORT---------------------        
        EXTERN  HardFault_Handler
        EXTERN  UsageFault_Handler
        EXTERN  __vector_table
        EXTERN  led_loop
        EXTERN  dbg_error_led
        EXTERN  LSCAN2_LEDToggle
        EXTERN  TSV_TIMER_IRQ_HNDLR
        EXTERN  svc_find_by_handle
        
        //--------------------------
        
        EXTERN  ccfree //void  ccfree(void* m)//������������ ������
        
        EXTERN  _proot_     //root context*
        EXTERN  _pend_
        
        EXTERN  pctcon      //������� �����     
        EXTERN  tswFLdis0
        //C funcs import
        EXTERN _check_for_sync
        EXTERN core_debug
        
//-----------EXPORT----------------------        
        PUBLIC PendSV_Handler
        PUBLIC sub_switch_context_to_R0_
        PUBLIC select_next_SW_thread
        PUBLIC sub_retIT_R0_thrd_EX_RETURN
        PUBLIC _psw_switch_threads_context_to_R0
//-----------------------------------------  
        PUBLIC _pend_thread_ //��� ��������������� ������������


//======================================================================
        THUMB //! offset+1 !
//====================================================================== 
        SECTION OS_CORE_DATA:DATA:ROOT(2)    //ALIGN 4
        
_pend_thread_         DCD   0   //��������� �����.���� �� 0,��������� ��� ������������.
//                      DCD   sub_retIT_R0_thrd_EX_RETURN
//=========================����� ������ ================================ 
        SECTION OS_CORE:CODE(2) 
        
#if CORE_DBG_OPERATION==1
//��������� �������� ����. ����������� ��� �������.
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
//------------------------------------------------------------------------------
PendSV_Handler:
        _save_op 46 
      /*          
        //� ����� ����� �����?
        MOV R3,LR
        AND R3,R3,#(EXC_RET_BASE|EXC_RET_THREAD)
        CMP R3,#(EXC_RET_BASE|EXC_RET_THREAD) //return to thread mode=no errors
        BEQ _psv_it_cont_lp0 //_psv_it_tst_ok
          MOV R0,#(ERR_EXC_RET)
          BL dbg_error_led //ERROR
        */  
 /*     
_psv_it_tst_ok:    
        LDR   R0, =pctcon
        LDR   R0, [R0]
        CMP R0,#0
        BNE.N _psv_it_cont_lp0 
        
_psv_it_dbg_lp0:
        B.N _psv_it_dbg_lp0  //debug loop
*/        
_psv_it_cont_lp0:
        //-------DEBUG-------
        //LDR R2, [R0, #offs_id] //dbg
        //ISB
        //DSB
        //MRS R2, PSP       //PSP->R0 (bit set)
        //STR R2, [R0,#offs_tstack_PSP]
        //������� ���������: � ����� PSR PC LR R12 R3 R2 R1 R0
        //LDM R12, {R0-R3}
        //LDR R3, [R12,#24]    //������� PC �� �����
        //LDR R3, [R12,#20]    //������� LR �� �����
        //-------------------
  //R1=current priority R0=next thread R2,R3-temp
  PUSH {LR}
  BL select_next_SW_thread
  POP {LR}
    
  CMP  R0,#0
  BEQ _psw_t_ret //����� �� ������
  
  LDR R1,=pctcon
  LDR R1,[R1]
  CMP R1,R0
  BEQ   _psw_t_ret //�������==�������

_psw_switch_threads_context_to_R0:          
  //=========����� �������� ������(��������)============
  /*
   ������������ ������ � ����� �� ����������.
   ������������ � ����������� PendSV,SVcall,������� Sleep � ��.
   */

  //PSR PC LR R12 R3 R2 R1 R0
  BL sub_switch_context_to_R0_ //��������� �������� � �������� ����� �������� ���������
        
//****������ �������� ���������, EXC_RETURN � ������� � ����� �� ����������*****        
sub_retIT_R0_thrd_EX_RETURN: 
  //---------------�������� ������ �����----------------
#if ENABLE_STACK_CHECK_DEBUG == 1
        LDR R2, [R0, #offs_id] //dbg
        LDR R12,[R0,#offs_mode]
        MRS R3, PSP //PSP val
        LDR R1, [R0,#offs_tstaRAM_PSP] //������ �������,������ �������
        CMP R3, R1
        IT GE
        BGE _psw_check_low_lim_PSP //R3>=low stack RAM limit==NO ERROR
        
_pswclo1: B _pswclo1
          //MOV R0,#(ERR_LO_STACK_LIM) //R0<RAM low
          //BL dbg_error_led //ERROR R0<RAM low
          
_psw_check_low_lim_PSP: 
        //LDR R3,[R0,#offs_tstack_PSP]
        MRS R3, PSP
        LDR R2,[R0,#offs_tstsize_PSP]
        LDR R1,[R0,#offs_tstaRAM_PSP] //������ �������,������ �������
        ADD R1,R1,R2 //����� �������(low_stack_RAM + size_stack),������� �������
        SUB R1,R1,#1
        CMP R1,R3 //R1>R3 ?
        IT  HI
        BHI _psw_set_EX_RETURN //no err , R3=PSP < up limit=R0
_pswclo2: B _pswclo2
          //MOV R0,#(ERR_UP_STACK_LIM) //R0>RAM up
          //BL dbg_error_led //ERROR R0>RAM up
#endif          
   //-----------���� � ��������.������������ ������---------   
   //CONTROL : 1=NOT PRIV 2 = PSP 4=USE FPU
_psw_set_EX_RETURN:  
    
        LDR   R0, =pctcon
        LDR   R0, [R0]
        //LDR   R1, [R0, #offs_id] //dbg
        //------EXC_RETURN--------
        LDR R1,[R0,#offs_mode]
        //PRIV? (CONTROL 0=PRIV/1=NO_PRIV)          
        MRS R3,CONTROL 
        AND R3,R3,#(~7) //1=NPRIV 2=PSP 4=FPU
        /*
Bit 1 SPSEL: 
Active stack pointer selection. Selects the current stack:
0: MSP is the current stack pointer 
1: PSP is the current stack pointer.
In Handler mode this bit reads as zero and ignores writes. The Cortex-M4 updates 
this bit automatically on exception return.

��� CONTROL[0] �������� ��� ������ ������ � �������r��������� ������- 
���. ����� ������������ ���������� �� ���������������� ������� ������� 
������������ ������ �������� � ������� ��������� ������� � r�������� �����- 
����� � ��������� ��������� ���������� � ����������� ���r� ����������. 
*/
        ANDS R2,R1,#TM_NPRIV
        IT NE 
          ORRNE R3,R3,#1 //NPRIV
        /*
        ANDS R2,R1,#TM_PSP
        IT NE 
          ORRNE R3,R3,#2
 
        ANDS R2,R1,#TM_FPU
        IT NE 
          ORRNE R3,R3,#4        
        */
        MSR CONTROL,R3
        
        MOV.W R3,#0 //ZERO      
        //FPU?
        ANDS R2,R1,#(TM_FPU) //0==no FPU in use 
        IT EQ
        ORREQ R3,R3,#(EXC_RET_NOFPU) //16=not use
       
        ANDS R2,R1,#(TM_PSP)
        IT NE
        ORRNE R3,R3,#(EXC_RET_PSP) //4=PSP ,0=MSP
        
        ORR R3,R3,#(EXC_RET_BASE|EXC_RET_THREAD)//to thread
 
        //������� � ����� ������
       
               
        //LDR R1, [R0, #offs_id] //dbg
        //LDR R2, [R0,#offs_tstack_PSP]
        //ADD.W R1,R0,#(offs_R4_R11_regs) //���������� �� ����� ������ ���������
        //LDMIA.W R1!,{R4-R11} //������������ ��������
        //MSR PSP,R2
        //ISB
        
        MOV LR,R3
        //===============================
        //LDR R12, [PSP,#24]    //������� PC �� �����
        //CMP R12,#0        
//ELOOP:  BEQ.N ELOOP //��� ������ ���������� !
        
_psw_t_ret:
  
        BX LR //������� � ����� ����������� /������
//===========����������� ������ ������ ��� ������������===========================
/*
������� ��������� ���������� ������ � ������ ����� ��� ���� ����������� ���
������������� ������ (����������� ��� ��������� �������� �������������).
���������� ����� ������� ������� ���� � ���������� ���������
(��������,Sleep ��� ������ ������).
�����,�� ��������� �������� �� �������������,
�� ����� ���� ������ ��� ������������ ������� ����������.

R1=current priority R0=thread (0, ���� ����� �� ������) R2,R3-temp 
*/
select_next_SW_thread:
  _save_op 47 
  PUSH  {R1-R3,LR}
  
  LDR       R0, =pctcon
  LDR       R0, [R0]
  CMP       R0,#0
  BEQ       _ssw_t_ret //pctcon==0 !!!==return
  LDR       R1, [R0, #offs_mode]
  MOV       R3,R1
  UBFX      R1, R1, #3, #5 //���� � 3 �� 7 =priority
  AND.W     R3,R3,#(TCRITSECT_Msk)
  CBZ.N     R3,_ssw_t_check_pend_correct //PC �� � ����������� ������
  AND       R0,R0,#0
  B.N       _ssw_t_ret //Crit.Sect
  
  //--------------------
_ssw_t_check_pend_correct:  //�������� �� ������������ ���������
  LDR       R2, =_pend_thread_
  LDR       R3,[R2]
  CBZ.N     R3,_ssw_t_ld_next //��� _pend_thread_
  //---����� � ������ ����-------
  MOV R0,R3 //R3->R0
  BL svc_find_by_handle
  CBNZ.N R0,_ssw_t_check_pend_equ //������
  B _ssw_t_clear_pending //�������� � ����� ���������
  //-----------------------------
_ssw_t_check_pend_equ:
  LDR       R0, =pctcon
  LDR       R0, [R0]
  LDR       R3, [R2]
  CMP       R3,R0 //��� �������?
  BNE.N     _ssw_t_ld_pend //NE== ������ ������������,��������� pend

_ssw_t_clear_pending:
  LDR       R0, =pctcon
  LDR       R0, [R0]
  //�������� pend,���������� �������
    AND       R3,R3,#0
    STR       R3,[R2]
    B.N       _ssw_t_ld_next //��� _pend_thread_
    
  //�������� _pend_thread_,���������� ���������  
_ssw_t_ld_pend:  
  //LDR       R0, =pctcon
  //LDR       R0, [R0]
  MOV.W R0,#0
  STR       R0, [R2] //��������� ������� ����� ��������� � pend
  //2 = ������ ������ ������������
  LDR R2,=tswFLdis0
  LDR R1,[R2]
  ORR R1,R1,#2//2 = ������ ������ ������������
  STR R1,[R2] //������ �������� � _pend_thread_
  
  MOV       R0,R3 //R3->R0 ��������� ���������
  LDR       R1, [R0, #offs_mode]
  UBFX      R1, R1, #3, #5 //���� � 3 �� 7 =priority
  B         _ssw_t_ld_ch_synclist    //�������� ������ �� �������������
  //-----------------------------------------------
_ssw_t_ld_next:
  LDR R2,=tswFLdis0
  LDR R3,[R2]
  ORR R3,R3,#1//1 = ������ ������������
  STR R3,[R2] //������ tswFLdis0
  //�������� ������� ��� �������������� ������������ (���������� ����������!)
  LDR.N R2, =(TSV_TIMER_BASE_ADR+TIMx_SR_OFFSET)
  LDRH  R3, [R2]
  AND   R3, R3, #1 //TIM_SR_UIF
  
  PUSH {R0-R3,R12,LR}
  CMP   R3,#1
  IT EQ
  BLEQ   TSV_TIMER_IRQ_HNDLR
  POP  {R0-R3,R12,LR}
  
  LDR R2,=tswFLdis0
  LDR R3,[R2]
  AND R3,R3,#0xFFFFFFFE //����� ��������
  STR R3,[R2] //������ tswFLdis0
  //-------------
  LDR       R0, [R0, #offs_next]
  CBNZ.N    R0,_ssw_t_check
  //--------------------------------------
_ssw_t_dec_prio:
  //if(prio==-1)prio=31;
  CMP       R1, #-1 
  ITTE      EQ                //==-1
    MOVEQ     R1,#31          //������������ ���������
    BEQ.N     _ssw_t_ld_root  //begin for root
  SUBNE     R1,R1,#1          //else prio--
  //--------------------------------------  
_ssw_t_ld_root: 
  LDR.N     R0, =_proot_
  LDR       R0,[R0]
  CBNZ      R0,_ssw_t_check  //OK
  B         _ssw_t_ret     //empty root!!!
  //--------------------------------------
_ssw_t_check:
  CMP       R0,#0
  BEQ       _ssw_t_dec_prio //CTX==0 -> dec prio,load root,continue
  //---�������� ����---
  LDRH      R2, [R0, #offs_ty]
  CMP       R2,#(TY_THREAD)
  BNE.N     _ssw_t_ld_next 
  
  LDR       R2, [R0, #offs_mode]
  //�������� suspended
  AND       R3,R2,#(TM_CREATE_SUSPENDED)
  CMP       R3,#0
  BNE.N     _ssw_t_ld_next
  //�������� ����������
  UBFX      R2, R2, #3, #5 //���� � 3 �� 7
  //if(cprio==prio)//��������� ������ �����
  CMP       R2,R1
  BNE.N     _ssw_t_ld_next
  
_ssw_t_ld_ch_synclist:
  //�������� �� �������� � �������� �������������,�������� sleep (C)
  PUSH {R0,R1}
  BL _check_for_sync
  MOV R2,R0
  POP {R0,R1}
  CMP R2,#0
  BEQ.N _ssw_t_ld_next //������������� �� ��������(���������� ��������� ����������)
//������� ��������� ���������� ������ � ������ ����� ��� 0 ��� ������������� ������.  
_ssw_t_ret: 
  POP {R1-R3,PC} //output:R0=selected thread
  
//=====================����������� �� ����� R0==================================
sub_switch_context_to_R0_: 
    _save_op 48 
   PUSH {R1-R3,R12,LR}
   //R0==new ctx, R1=current ctx
   //��������� ��������� �����
   LDR R2,=pctcon  //��������� �� ��������
   LDR R1,[R2]     //�������� ���������
   CMP R1,R0
   BEQ.N swr0_ret  //��� ������� !
   
   LDR.W R3,[R1,#offs_mode] 
   AND.W R3,R3,#(TCRITSECT_Msk)
   CMP  R3,#0
   IT   NE
     MOVNE R0,R1    //��������� �������
     BNE.N swr0_ret //PC � ����������� ������ !
   //------------------------------
   //���������� ��������� FPU - ��������������, ��� ���������� !
   //------------------------------
   //��������� ���� �������� ���������
   MRS   R3, PSP
   STR   R3, [R1,#offs_tstack_PSP]
   
   ADD.W R1,R1,#(offs_R4_R11_regs) //���������� �� ����� ������ ���������
   STMIA.W R1!,{R4-R11} //��������� ��������

   ADD.W R1,R0,#(offs_R4_R11_regs) //���������� �� ����� ������ ���������
   LDMIA.W R1!,{R4-R11} //������������ ��������

   //������������ ���� ���������� ���������
   LDR   R3, [R0,#offs_tstack_PSP]
   MSR   PSP,R3
   ISB
   CMP R3,#0
psw_ERR_DLOOP: BEQ.N  psw_ERR_DLOOP 
   
   LDR R12, [R3,#24]    //������� PC �� �����
   //ANDS R1, R12,#1       //THUMB ?
   //BEQ.N dloop1
   CMP R12,#0 //�������� PC
   BNE.N sswct_save_f
   
dloop1:
    B dloop1
   //------------------------------    
sswct_save_f:   
   //����� �������
   LDRH R1,[R0,#offs_flags] 
   ORR  R1,R1,#(CF_SIGNAL)
   STRH R1,[R0,#offs_flags] 
   
   STR.W R0,[R2] //�������� ����� �������� �������� ���������
   
swr0_ret:
   
   POP {R1-R3,R12,LR}
   BX LR
//==============================================================================

        END
//==================@ MW dem1305 2017====dem1305@yandex.ru===========EOF