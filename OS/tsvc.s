/*==============================================================================
                           svc call 
                        @dem1305 15.04.2017
==============================================================================*/
#include "core_def.h"

//System handler priority register 2 (SHPR2) ==SVcall prio.


//----------------------------------------------------------------------------//
        MODULE  SVC
        
        SECTION OS_CORE:CODE(2) 
                
#if CORE_DBG_OPERATION==1
        EXTERN _dbg_op //(CORE_DBG_LEN) ����
        EXTERN _dbi
#endif
//-----------IMPORT---------------------        
        EXTERN  HardFault_Handler
        EXTERN  __vector_table
        EXTERN  led_loop
        EXTERN  dbg_error_led
        EXTERN  LSCAN2_LEDToggle
        EXTERN  select_next_SW_thread
        EXTERN  sub_switch_context_to_R0_
        
        EXTERN  PendSV_Handler
        EXTERN  mfree //void  mfree(void* m)//������������ ������
        
        //EXTERN  svc_new_thrd //����� �����
        EXTERN  tfunc_wrapper //������� ������� ������
        EXTERN  twrap_del     //�������� ������� ������
        //---------- 
        EXTERN  topMSP
        EXTERN  pctcon //������� ��������
        
        EXTERN  _proot_       //root context*
        EXTERN  _pend_
        EXTERN  _otot_
        //---------- 
        EXTERN  svc_new_thrd  //����� �����  
        
        EXTERN  tswFLdis0     //����� ������������
        //--C funcs import-----
        EXTERN  _rls_object
        EXTERN  svc_close_hndl
        EXTERN  core_cut_CoreObj
        EXTERN  core_cut_CoreObj_no_find
        EXTERN  core_add_CoreObj
            // sys timer func
        EXTERN  svc_RestartSystemTimer
        EXTERN  svc_PauseSystemTimer
        EXTERN  svc_ResumeSystemTimer
        EXTERN  svc_isSystemTimerCounterNull
        EXTERN  svc_isSystemTimer
        EXTERN  svc_DeleteSystemTimer
        EXTERN  core_scan_tasklist
        
        //EXTERN core_check_stack_pointer
        EXTERN core_debug
        /*
        ������� �� ���������� � ��������� EX_RETURN 
        � ���������� ����������� SP
        */
        EXTERN sub_retIT_R0_thrd_EX_RETURN 
        
//-----------EXPORT------------------------  
        PUBLIC  SVC_Handler
        PUBLIC  __in_R0_current_TOP_stack_ctx
        PUBLIC  __out_R0_to_current_stack_ctx
        PUBLIC  __out_R0_to_stack_R1_ctx //����� R0 � ���� ���������� ��������� ������ R1
        PUBLIC  svc_find_by_handle
        PUBLIC  svc_delete_thread
        
        //PUBLIC  svc_add_core_obj          //1 func connect core object
        PUBLIC  svc_set_not_privileged    //4 func set npriv access
        PUBLIC  svc_set_privileged        //5 func set priv access
        PUBLIC  svc_new_thread
//-----------------------------------------  
        //ONLY CORE ACCESS
        PUBLIC    svc_get_thread_id         //10 func get thread id by HTHREAD (in R0=context)
        PUBLIC    svc_get_thread_handle     //11 func get thread context by ID (in R0=id)
        PUBLIC    svc_change_core_obj_flags //21        


//======================================================================
        THUMB //! offset+1 !
//====================================================================== 
        SECTION .rodata:CONST:ROOT(2)   //ALIGN 4
        
SVC_Cnt         EQU    (SVC_End-SVC_Table)/4
SVC_Count       DCD    SVC_Cnt
//------------------ 
SVC_Table

                DCD     svc_delete_thread         //0 func delete thread
                
                DCD     core_add_CoreObj          //1 func connect core object
                
                DCD     svc_sw_thread             //2 func switch thread
                DCD     svc_get_set_thread_prio    //3 func set thread prio
                
                DCD     svc_set_not_privileged    //4 func set npriv access
                DCD     svc_set_privileged        //5 func set priv access
                
                DCD     svc_set_sleep             //6 func set sleep value
                
                DCD     svc_set_suspend           //7 func set suspend mask
                
                DCD     svc_find_by_handle        //8 func find thread (in R0=context)
                DCD     svc_find_by_ID            //9 func find thread (in R0=id)
                
                DCD     svc_get_thread_id         //10 func get thread id by HTHREAD (in R0=context)
                DCD     svc_get_thread_handle     //11 func get thread context by ID (in R0=id)
                
                DCD     svc_EnterCriticalSection  //12 func disable sw thread
                DCD     svc_LeaveCriticalSection  //13 func enable sw thread
      //������/���������� ���������� ���������� ������������ ����������
                DCD     svc_disable_SW            //14 disable  pendSV IT 
                DCD     svc_enable_SW             //15 enable   pendSV IT
                
                DCD     svc_push_SWflags_R0_disable_SW //16 ��������� ��������� � R0 � ��������� ������������
                DCD     svc_pop_SWflags_R0_SW          //17 ������������ ��������� �� R0
                
                DCD     core_cut_CoreObj             //18 �������� ������ ���� �� ������
                
                DCD     svc_save_thread_sync_parameters //19 ������ ���������� �������������
                
                DCD     svc_ReleaseObject          //20 ���������� �������
                DCD     svc_change_core_obj_flags  //21 �������� ����� ������� ����
                
                DCD     svc_close_hndl               //22 ���������� ������� �����������
                
                DCD     svc_find_and_ret_HANDLE    //23 ����� �������,������� HANDLE � ���� �������� ������ ��� �����
                //--------SYS TIMER-------------------
                DCD svc_RestartSystemTimer      //#24
                DCD svc_DeleteSystemTimer       //#25
                DCD svc_PauseSystemTimer        //#26
                DCD svc_ResumeSystemTimer       //#27
                DCD svc_isSystemTimerCounterNull//#28
                DCD svc_isSystemTimer           //#29
                //------------------------------------
                DCD core_cut_CoreObj_no_find    //30
                
                DCD svc_new_thread              //31
                
                //�������������� ������������ ��������� ������ �� R0
                DCD svc_sw_thread_to_R0         //#32
                
                //------------------------------------
                DCD svc_scan_tasklist           //33 �������������� ������ �����
                //------------------------------------
    
SVC_End
//------------------       

//------------------------------------------------------------------------------  
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
SVC_Handler:
        _save_op 49 
        /*
        //� ����� ����� �����?
        MOV R3,LR
        AND R3,R3,#(EXC_RET_BASE|EXC_RET_THREAD)
        CMP R3,#(EXC_RET_BASE|EXC_RET_THREAD) //return to thread mode=no errors
        BEQ svc_it_tst_ok //svc_it_tst_ok_0 
          MOV R0,#(ERR_EXC_RET)
          BL dbg_error_led //ERROR
          */
/*        //-------------------------  
svc_it_tst_ok_0:        
        LDR.W   R0, =pctcon
        LDR   R0, [R0] 
        CMP R0,#0
        BNE.N _svc_it_cont_lp0 
        
_svc_it_dbg_lp0:
        B.N _svc_it_dbg_lp0  //debug loop
        
_svc_it_cont_lp0:        
        MRS R2, PSP       //PSP->R0 (bit set)
        STR R2, [R0,#offs_tstack_PSP]
        ISB
        DSB*/
        //-------------------------  
svc_it_tst_ok:        
        TST   LR, #4        //EXC_RETURN (bit 2)?
        ITE   EQ            //if Z - MSP else PSP
        MRSEQ R12, MSP       //MSP->R0
        MRSNE R12, PSP       //PSP->R0 (bit set)
        //������� ���������: � ����� PSR PC LR R12 R3 R2 R1 R0
        LDM R12,{R0-R3}
        
        LDR  R12, [R12,#24]    //������� PC �� �����
        LDRB R12,[R12,#-2]    //������� SVC ���������
                
        LDR     LR,=SVC_Count
        LDR     LR,[LR]
        CMP     R12,LR
        BHS     SVC_Dead                ; Overflow(SVC num >= SVC_Count)
        LDR     LR,=SVC_Table
        LDR     R12,[LR,R12,LSL #2]     ; Load SVC Function Address [base+num*4]
        BLX     R12                     ; Call SVC Function
        
        LDR.W   R0, =pctcon
        LDR     R0, [R0]
        B       sub_retIT_R0_thrd_EX_RETURN  //������ �������� � ����� �� ���������� 
      //--------------------------          
SVC_Dead
        B       HardFault_Handler                ; None Existing SVC        
//====================�������� ������ (��������)================================ 
//�������� � R0==��������� �������� .������ 0 ��� ������(������� ����� ����)
//���� � R1 �� 0-��������� ������� ������������ � ����.
svc_delete_thread: //#0
    _save_op 50 
    PUSH  {R0-R3,LR}
    //-----------
    MOV R2,R1 //�������� �� ��������� � ����
    MOV R3,R0 //���������
    
    CBNZ.N R0, svd_dt_find //������� ���������=��������� �������
    LDR.W R0,=pctcon //��������� �������
    LDR R0,[R0]
    CMP R0,#0
    BEQ svd_ret //0 == ������ !-> return 0
    B.N svd_dt_check_type
    
svd_dt_find:
    //---����� � ������ ����-------
    BL svc_find_by_handle
    CBNZ.N R0,svd_dt_check_type //������
    B   svd_ret //������ !
    //-----------------------------
svd_dt_check_type:
    LDRH R1,[R0,#offs_ty]
    CMP R1,#TY_THREAD
    BEQ.N svd_dt_correct
    MOV R0,#0 //0 == ������ !-> return 0
    B   svd_ret //������ !
    
svd_dt_correct:
    
    LDR.W R1,=pctcon
    LDR R1,[R1]
    CMP R0,R1 
    IT  NE //��� ������� ?
     BNE svd_ccobj    //���,������ � ������������ ������
      //����������� ��������,���� ���������==�������
svd_scutr_loop:
      BL select_next_SW_thread
      LDR.W R1,=pctcon
      LDR R1,[R1]
      CMP R0,R1 
        BEQ.N svd_scutr_loop  //���������� ! ��������� (�������� Sync)
          CBZ R0,svd_ret //����� �� ������
        //------�������� ��������------
        BL sub_switch_context_to_R0_
        //-----------
        LDR R2,[R0,#offs_id] //DBG ����� ��������
        //-----------
        MOV R2,#0 //�� �������� ��������� � ����
        //-----------------------------
        
        
svd_ccobj:
    MOV R0,R3//������ ��������
    
    PUSH {R2,R3}
    BL core_cut_CoreObj_no_find //�������� ������ ����
    POP {R2,R3}
    CBZ.N R0,svd_ret //0 == ������ !
    
    //free memory
    LDR R0,[R3,#offs_tstaRAM_PSP]
    PUSH {R2,R3}
    BL mfree //���������� ������ �����
    POP {R2,R3}
    
svd_ccobj_free_ctx:    
    MOV R0,R3
    
    PUSH {R2,R3}
    BL mfree //���������� ������ ��������� ���������
    POP {R2,R3}
    
    MOV R0,#1 //TRUE==OK
    DSB
    
svd_ret
    MOV R1,R2
    CMP R1,#0
    IT  NE
      BLNE  __out_R0_to_stack_R1_ctx
    POP {R0-R3,PC}
//========================�������� ���������==================================== 
//#3 R0=pctx R1=prio �������� ����� ���� R2=����� ��� ������ � ���� R3=0==get/1==set
//int svc_get_set_thread_prio(HTHREAD pCTX_R0,uint8_t priority_R1,HTHREAD ht_R2,uint8_t bget_set_R3);//+
svc_get_set_thread_prio:      
        _save_op 51 
        //R0=pctx R1=prio
        PUSH {R0-R3,LR}
                
        BL svc_find_by_handle
        CBNZ R0,svccp_set //������
        MOV R0,#(-1) //error
        B.N svccp_ret
        CMP R3,#0
        BNE.N svccp_set
        //------get------
        LDR R2,[R0,#offs_mode]
        AND R0,R2,#(TPRIO_Msk)
        LSR R0,R0,#(TPRIO_BIT_Pos)
        B.N svccp_ret
        //---------------
svccp_set:    
        AND R1,R1,#31 //�����
        LSL R1,R1,#(TPRIO_BIT_Pos)
       
        LDR R2,[R0,#offs_mode]
        AND R2,R2,#(~TPRIO_Msk)
        ORR R2,R2,R1
        STR R2,[R0,#offs_mode]
        LSR R1,R1,#(TPRIO_BIT_Pos)
        MOV R0,R1 //no err
        
svccp_ret:
        POP {R0-R3}
        MOV R1,R2
        CMP R1,#0
        IT  NE
          BLNE  __out_R0_to_stack_R1_ctx
        POP {PC}
//=========������������ ��������� ������ (��������)======   
svc_sw_thread: //#2
    _save_op 52 
    PUSH  {LR}
    //--------------------
    //R0-R3 ���������.
    //--------------------
    BL    select_next_SW_thread
    CMP   R0,#0
    BNE.N svc_sw_sw_to_R0 //����� ������
    //--------------------   
svc_sw_inf_loop: B svc_sw_inf_loop
    //--------------------  
svc_sw_sw_to_R0:
    BL    sub_switch_context_to_R0_ 
    
svc_sw__ret:
    POP   {PC}
//=========������� ��������� �������������������� ������� ������================
svc_set_not_privileged: 
    _save_op 53 
    PUSH {R0-R2}
    
    LDR.W R1,=pctcon
    LDR   R0,[R1]
    LDR   R2,[R0,#offs_mode] //COREOBJ
    ORR   R2,R2,#(TM_NPRIV)
    STR   R2,[R0,#offs_flags]
    
    MRS R0,CONTROL
    ORR R0,R0,#1
    MSR CONTROL, R0 
    POP {R0-R2}
    BX LR
//=========������� ��������� ������������������ ������� ������==================
svc_set_privileged:
    _save_op 54 
    PUSH {R0-R2}
    
    LDR.W   R1,=pctcon
    LDR   R0,[R1]
    LDR   R2,[R0,#offs_mode] //COREOBJ
    AND   R2,R2,#(~TM_NPRIV)
    STR   R2,[R0,#offs_flags]
    
    MRS R0,CONTROL
    AND R0,R0,#(~1)
    MSR CONTROL, R0 
    POP {R0-R2}  
    BX LR
//=================== ������������� ����� R0 �� R1 ms ========================== 
/*
��������� ���������� ����� ���� PSP/MSP, �������� �������� ���.
���������� ���������� (���������) ���������� ��� ����� � ������.
���� R0==0-��������� ��� �������� ������.R1==0-�������� ����� ���������� ������
*/
svc_set_sleep: //svc #6
    _save_op 55 
    PUSH {R1-R3,LR}
    //-------
    BL svc_find_by_handle
    CMP R0,#0
    IT EQ 
    POPEQ {R1-R3,PC} //CBZ.N R0,svsl_ret
      
    LDR.W R2,=tswFLdis0
    LDR R3,[R2]
    ORR R3,R3,#2 //������� ������ ������������ � ����� ������
    STR R3,[R2]
    
    STR    R1,[R0,#offs_wait_ms]  //������ ��������� ��������
    
    //-------------------------------------
    LDRH   R2,[R0,#offs_flags] //core object
    AND    R2,R2,#(~CF_TIMEOUT)  //�������� ����� ~(CF_TIMEOUT|CF_SIGNAL)
    AND     R2,R2,#(~CF_SIGNAL)
    STRH   R2,[R0,#offs_flags] 
    //������������ ���������� � ����� ������
    BL select_next_SW_thread
  
    CMP   R0,#0
    IT    EQ
    POPEQ {R1-R3,PC} //BEQ svsl_ret //����� �� ������
  
    LDR.W   R1,=pctcon
    LDR   R1,[R1]
    CMP   R1,R0
    IT    EQ
    POPEQ {R1-R3,PC} //BEQ   _psw_t_ret//�������==return !!!
    //=========����� �������� ������(��������)============
    /*
     ������������ ������ � ����� �� ����������.
     ������������ � 
     ���������� PendSV,
     ������� Sleep,
     */
    //PSR PC LR R12 R3 R2 R1 R0
    BL sub_switch_context_to_R0_ //��������� �������� � �������� ����� �������� ���������
    //������ �������� ���������, EXC_RETURN � ������� � ����� �� ����������  
    POP {R1-R3,PC} 
//================= ��������� ����� suspend R1 ������ R0 ======================= 
//���� R0==0-��������� ��� �������� ������
//uint32_t  call_set_suspend(HTHREAD hthrd_R0, uint32_t suspendMask_R1,HTHREAD ht_R2);
svc_set_suspend: //svc #7
  _save_op 56
  
  PUSH {R1-R2,LR}
  BL svc_find_by_handle
  CBNZ R0,svss_set_suspmask //������
  MOV R0,#(-1) //error
  B.N svss_ret
  //---��������� ����---
svss_set_suspmask:
  
  LDRH   R2,[R0,#offs_flags] //core object
  CMP    R1,#(TM_CREATE_SUSPENDED)
  ITT    EQ
    ANDEQ  R2,R2,#(~CF_SIGNAL)  //����� �� �������,���� TM_CREATE_SUSPENDED
    STRHEQ R2,[R0,#offs_flags] 
    
  AND R1,R1,#(TM_CREATE_SUSPENDED)
  LDR R2,[R0,#offs_mode]
  AND R2,R2,#(~TM_CREATE_SUSPENDED)
  ORR R2,R2,R1
  STR R2,[R0,#offs_mode]
  AND R0,R0,#0 //no errors
  
svss_ret:
  POP {R1-R2}
  MOV R1,R2
  CMP R1,#0
  IT  NE
     BLNE  __out_R0_to_stack_R1_ctx
  
  POP {PC}
//============����� ������ �� R0=POBJECT(�������� ���������)=====================
//(��� R0==0 � R0 ����� �������� ������� �������� ������,
//������ ��������� (����� ���������) � R0 , ��� 0,���� �� ������
svc_find_by_handle: //#8
  _save_op 57
  
  PUSH {R1-R3,LR}
  //����� �� ��������� R0
  CBNZ.N R0,svfbh_check_begin
  LDR.W R0,=pctcon //�������
  LDR R0,[R0]
//����� ���������  
svfbh_check_begin:
    LDR.W R3,=_proot_ //���������
    LDR R3,[R3]   //��������
    
svfbh_loop_search:
    CBNZ R3,svfbh_cont //�� ����
    MOV R0,#0   //�� ������
    B.N svfbh_ret  //�����,���� ������ ��� (�� ������)
    LDR.W R1,=_pend_ //���������?
    LDR R1,[R1]    //��������
    CMP R3,R1   
    ITT EQ
    MOVEQ R0,#0   //�� ������
    BEQ.N svfbh_ret  //�����,���� ������ ��� (�� ������)
    
svfbh_cont:
    CMP R3,R0     
    BEQ.N svfbh_ret //������������� (R0==R3==��������)
    LDR R3,[R3,#offs_next]
    B.N svfbh_loop_search

svfbh_ret:
  POP {R1-R3,PC}
  
//============����� ������ �� R0=HANDLE(�������� ���������)=====================
//������ ��������� (����� ���������) � R0 , ��� 0,���� �� ������
//HANDLE call_find_handle (HANDLE ho_R0,HTHREAD ht_R1); #23 
svc_find_and_ret_HANDLE: //#23
    _save_op 58
    
  PUSH {R1-R3,LR}

//����� ��������� c _proot_
    LDR.W R3,=_proot_   //���������
    LDR R3,[R3]       //��������
    
svfrh_loop_search:
    CBNZ R3,svfrh_ret //�� ����
    MOV R0,#0         //�� ������
    B.N svfrh_ret     //�����,���� ������ ��� ��� 0 (�� ������)
    LDR.W R1,=_pend_    //���������?
    LDR R1,[R1]       //��������
    CMP R3,R1   
    ITT EQ
      MOVEQ R0,#0       //�� ������
      BEQ.N svfrh_ret   //�����,���� ������ ��� (�� ������)
    
svfrh_cont:
    CMP R3,R0     
    BEQ.N svfbh_ret   //������������� (R0==R3==HANDLE)
    LDR R3,[R3,#offs_next]
    B.N svfrh_loop_search

svfrh_ret:
  POP {R1-R3}
  CMP R1,#0
  IT  NE
     BLNE  __out_R0_to_stack_R1_ctx
  POP {PC}
//=================����� ����� �� R0=HTHREAD->id================================
//(��� R0==0 � R0 ����� �������� ������� ��������->id,
//������ ��������� (����� ���������) � R0 , ��� 0,���� �� ������
svc_find_by_ID: //#9
  _save_op 59 
  
  PUSH {R1-R3,LR}
  //����� �� ��������� ID � R0
  CBNZ.N R0,svfbi_check_begin
  LDR.W R0,=pctcon //�������
  LDR R1,[R0]    //�������� ���������
  LDR R0,[R1,#offs_id]
//����� ���������  
svfbi_check_begin:
    LDR.W R3,=_proot_ //���������
    LDR R3,[R3]   //��������
    
svfbi_loop_search:
    CBNZ.N R3,svfbi_cont //�� ����
    MOV R0,#0   //�� ������
    B.N svfbi_ret  //�����,���� ������ ��� (�� ������)
    
svfbi_cont:
    LDR R2,[R3,#offs_id]
    CMP R2,R0  
    ITT NE
    LDRNE R3,[R3,#offs_next]
    BNE.N svfbi_loop_search
    //������������� (R0==R2==id,R3=context)
    MOV R0,R3 //context value
    
svfbi_ret:
  POP {R1-R3,PC}
//================= ������� ID ������ R0 ======================================= 
//���� R0==0-��������� � R0 HTHREAD �������� ������ (����� �� HTHREAD)
//����/����� ����� ����
//���� ����� �� ������, ������ 0
//int svc_get_thread_id(HTHREAD ht_R0,HTHREAD ht_outstack_R1);
svc_get_thread_id: //#10
  _save_op 60
  
  PUSH {LR}
  
  BL svc_find_by_handle
  CBNZ R0,svgti_get_id //������
  //R==0 == �� ������
  B.N svgti_ret
  
svgti_get_id: //return ID
  LDR R0,[R0,#offs_id]
  
svgti_ret:
  CMP R1,#0
  IT  NE
     BLNE  __out_R0_to_stack_R1_ctx
  POP {PC}
//================= ������� HTHREAD ������ � ID==R0 ============================
//���� R0==0-��������� � R0 ID �����e�� ������ (����� �� ID)
//����/����� ����� ����
//���� ����� �� ������, ������ 0
//HANDLE svc_get_thread_handle(int id,HTHREAD ht_outstack_R1);)//return -1=error
svc_get_thread_handle: //#11
  _save_op 61
  
  PUSH {LR}
  BL svc_find_by_ID
  //R!=0 == ������
  CMP R1,#0
  IT  NE
     BLNE  __out_R0_to_stack_R1_ctx
  POP {PC}
//=============����������� ������,������/���������� ������������================   
svc_EnterCriticalSection: //12
    _save_op 62
    
    PUSH {R0,R1}
    LDR R0,=pctcon //�������
    LDR R0,[R0]
    LDR.W R1,[R0,#offs_mode] 
    ORR.W R1,R1,#(TCRITSECT_Msk)
    STR.W R1,[R0,#offs_mode] 
    POP {R0,R1}
    BX LR
svc_LeaveCriticalSection: //13
    _save_op 63
    
    PUSH {R0,R1}
    LDR R0,=pctcon //�������
    LDR R0,[R0]
    LDR.W R1,[R0,#offs_mode] 
    AND.W R1,R1,#(~TCRITSECT_Msk)
    STR.W R1,[R0,#offs_mode] 
    POP {R0,R1}
    BX LR
//=============������� ������������� ����,������/���������� ������������========  
svc_disable_SW: //14
    _save_op 64
    
    PUSH {R0,R1,LR}
    LDR R0,=tswFLdis0
    LDR R1,[R0]
    ORR R1,R1,#1 //1 = ������ ������������ !
    STR R1,[R0]
    POP {R0,R1,PC}
    
svc_enable_SW: //15
    _save_op 65
    
    PUSH {R0,R1,LR}
    LDR R0,=tswFLdis0
    LDR R1,[R0]
    AND R1,R1,#(~1) //����� ����� ������� ������������
    STR R1,[R0]
    POP {R0,R1,PC}
    
svc_push_SWflags_R0_disable_SW: //16 ��������� ��������� � R0 � ��������� ������������
    _save_op 66 
    
    PUSH {R1,R2,LR}
    LDR R1,=tswFLdis0
    LDR R0,[R1]
    ORR R2,R0,#1 //1 = ������ ������������ !
    STR R2,[R1]
    POP {R1,R2,PC}

svc_pop_SWflags_R0_SW: //17 ������������ ��������� �� R0
    _save_op 67 
    
    PUSH {R1,LR}
    LDR R1,=tswFLdis0
    STR R0,[R1]
    POP {R1,PC}
//=======����� �������� R0 �� ���������� � ���� ������ � ���������� R1==========
//� R0-�������� ��� ������,R1-�������� ������
//���� R1==0,�� ����� ����� � ���� �������� ������
__out_R0_to_current_stack_ctx: //����� � ���� �������� ������
    _save_op 68 
    
    PUSH {R1-R3,LR}
optsctx_ld_cur_t:
    MRS R2,PSP
    STR R0,[R2]  //R0 � ����� ������ �� �������
    POP {R1-R3,PC}
    
__out_R0_to_stack_R1_ctx: //����� R0 � ���� ���������� ��������� ������ R1
    _save_op 69 
    
    PUSH {R1-R3,LR}
    CMP R1,#0
    BEQ.N optsctx_ld_cur_t //0?=load current thread
    LDR R2,=pctcon
    LDR R3,[R2]
    CMP R1,R3
    BEQ.N optsctx_ld_cur_t //�������
    
    LDR R2,[R1,#offs_tstack_PSP]
    STR R0,[R2]  //R0 � ����� ������ �� �������
    POP {R1-R3,PC}
//=======��������� �������� ������� ����� ������ � ���������� R1================
//R1-�������� ������,���� R1==0,�� ��� ���� �������� ������
__in_R0_current_TOP_stack_ctx: //���� �������� ������
    _save_op 70 
    
    PUSH {R1-R3,LR}
iptsctx_ld_cur_t:
    MRS R0,PSP
    POP {R1-R3,PC}
    
__in_R0_stack_R1_ctx: //�������� � R0 ���� ���������� ��������� ������ R1
    _save_op 71
    
    PUSH {R1-R3,LR}
    CMP R1,#0
    BEQ iptsctx_ld_cur_t //0?=load current thread
    LDR R2,=pctcon
    LDR R3,[R2]
    CMP R1,R3
    BEQ.N iptsctx_ld_cur_t //�������
    
iptsctx_ld_stack_ptr:
    LDR R0,[R1,#offs_tstack_PSP]
    POP {R1-R3,PC}
//===========������ ���������� ������������� � �������� ������================== 
/*
��������� �������� ����� ���� ������ :
R0=SYNC_PAR*
typedef struct
{
  HANDLE   ctx;       //R1 �������� ������
  uint32_t flags;     //R2 ����� CF_WAIT_ALL|CF_WAIT_MULTIPLE
  HANDLE*  sync;      //R3 ������ ������ ������������� (������ ���������� �� �������)
  uint32_t sync_size; //R4 ���������� �������� �������������
  uint32_t wait_ms;   //R5 ����� �������� ����������� ��������� �������� �������������
}SYNC_PAR;
*/
svc_save_thread_sync_parameters: //svc #19
    _save_op 71
    //R0-R3 ��������� � IT SVcall !
    PUSH {R0-R5,LR}
    //R0 on top, R0=*SYNC_PAR
    
    LDM R0,{R1-R5}
    CMP R1,#0
    ITT EQ
    LDREQ R1,=pctcon
    LDREQ R1,[R1]
    
    STR R3,[R1,#offs_sync]
    STR R4,[R1,#offs_sync_size]
    STR R5,[R1,#offs_wait_ms]
    
    //-------------------------------------
    LDRH   R0,[R1,#offs_flags] //core object
    CMP    R5,#0
    IT     NE // !0
    ANDNE  R0,R0,#(~CF_SIGNAL)  //����� �� �������,���� wait_ms>0
    
    LDRH   R5,[R1,#offs_mode] //core object
    AND    R5,R5,#(TM_CREATE_SUSPENDED) 
    CMP    R5,#0
    IT     NE // !0
    ANDNE  R0,R0,#(~CF_SIGNAL)  //����� �� �������,���� TM_CREATE_SUSPENDED
    //-------------------------------------
    AND R0,R0,#(~(CF_TIMEOUT | CF_ABANDONED))
    AND R3,R2,#(CF_WAIT_ALL|CF_WAIT_MULTIPLE)
    AND R0,R0,#(~(CF_WAIT_ALL|CF_WAIT_MULTIPLE))
    ORR R0,R0,R3 //�����
    
    STRH R0,[R1,#offs_flags]
    
    POP  {R0-R5,PC}
//==============================================================================
/*
��������� �������� ����� ���� ������ :R0= object,R1=thread 
Mutex:����������� ���� ���
Semaphore: count++
������ TRUE,���� �������,���� ������ ����� ��� ������ � ����
BOOL      call_rls_object(PCOREOBJ pco_R0,HTHREAD ht_R1);
*/
svc_ReleaseObject: //svc #20
    _save_op 72
    //R0-R3 ��������� �� ����� ��� ����� � SVcall.
    PUSH {R0-R3,LR}
    BL   svc_find_by_handle //(R0)
    CBZ  R0,relobj_ret //�� ������
    //------
    PUSH {R1} 
    BL _rls_object //R0=TRUE/FALSE
    POP {R1}
    //------
relobj_ret:
    CMP R1,#0 //����� � ���� ������ R1 ?
    IT NE
      BLNE __out_R0_to_stack_R1_ctx
    POP {R0-R3,PC}
//====================�������� ����� ������� ����=======================
/* #21
HANDLE    call_change_core_object_flags(PCOREOBJ pco_R0,uint16_t flags_R1,uint8_t SET_RESET_R2,HTHREAD ht_R3);
 R0=������ ���� R1=(uint16_t)flags, R2=SET/RESET, R3=����� ��� ������.���� �� ������-������ �� �����.
 ��������� �������� � �����
 ������ ���������� � �����,���� �������.
 */
svc_change_core_obj_flags:
    _save_op 73
    //R0-R3 ��������� � SVcall ��� �����.
    PUSH {R0-R3,LR}
    BL svc_find_by_handle
    CBZ R0,svcchcof_ret //�� ������
    //------
    LDRH  R3,[R0,#offs_flags]
    CBZ.N R2,svcchcof_reset
    ORR   R3,R3,R1 //R3=R3 | R1
    B.N   svcchcof_save
    //------
svcchcof_reset:
    BIC   R3,R3,R1 //R3=R3 & ~R1
    //------
svcchcof_save:
    STR R3,[R0,#offs_flags]
    
svcchcof_ret:
    POP {R0-R3}
    
    CMP R3,#0 //����� � ���� ������ R3 ?
    ITT NE
      MOVNE R1,R3
      BLNE __out_R0_to_stack_R1_ctx
      
    POP{PC}
//==============================================================================
svc_new_thread: //31

    _save_op 74 
/*
 HANDLE    svc_new_thread(uint32_t args,HTHREAD ht_R1);//only core accept
 �������� ������ ������ (��������)  
 R0==param {+0:(uint32_t)pFunc,+4:(uint32_t)pArgs,+8:(uint32_t)pCTX}; (� �����)
 [17FPU regs] PSR PC LR R12 R3 R2 R1 R0   
 R1=����� ��� ������
*/
        //R0-R3 ��������� ��� ����� � ���������� SVcall
        PUSH {R0-R3,R12,LR}
        //���������� ������ ��������� //FFFFFFF8=8 align FFFFFFFC=4
        LDR R1,[R0,#8]  //��������� �� �������� pCTX
        LDR R3,[R1,#offs_tstack_PSP] //��������� SP
        AND R3,R3,#~7  // align down 8
        SUB R3,R3,#104 //-26 * 4 (with FPU)
        //----��������� ���������� ��������� FPU?--------;
        LDR R2,[R1,#offs_mode]
        ANDS R2,R2,#TM_FPU 
        BEQ svnt_push8regs //0==no FPU
          VMRS R2,FPSCR
          STR.W R2,[R3,#96] //1 reg FPSCR
    /*     MOV       LR,R3
          ADD.W     LR,LR,#32 //{S0-S15} �������� � �����
          VSTMIA.W  LR!,{S0-S15} //==VPUSH {S0-S15},16regs  
    */      
svnt_push8regs:
        //SUB R3,R3,#32 //CPU STACK SIZE
        //PSR-------���������� PSR(��� 9=������������)
        LDR R2,=SCB_CCR_REG//=0xE000ED14 = 8b align else 4b align
        LDR R2,[R2] //bit 9
        ORR R2,R2,#0x1000000 //24 bit(THUMB)
        //PUSH {R2} //PSR=0/0x200
        STR.W R2,[R3,#28] //1 reg FPSCR
        //PC----------------        
        //��� 0 ������ ���� ����������!(Thumb)
        LDR R2,=tfunc_wrapper
        STR.W R2,[R3,#24] //1 reg PC
        //LR----------------
        LDR R2,=twrap_del
        STR.W R2,[R3,#20] //1 reg LR=twrap_del
        
        PUSH {R0,R3}
        
        //other registers: R1=func R0=arg R2=ctx
        LDR R1,[R0]     //pFunc
        LDR R2,[R0,#8]  //PCX == thread context
        LDR R0,[R0,#4]  //args for pFunc
        
        MOV LR,R3
        STMIA.W LR!,{R0-R3,R12} //5 regs 
        
        POP {R0,R3}
          
        //���������� ������� �����
        LDR R1,[R0,#8]  //��������� �� �������� pCTX
        STR R3,[R1,#offs_tstack_PSP]     
        
        POP {R0-R3}
        //----����������� ������ � ������-----
        LDR R0,[R0,#8]  //��������� �� �������� CTX
//PCOREOBJ core_add_CoreObj(PCOREOBJ pobj,HANDLE ht_R1)//�������� � ������ ���� ������ ����
        BL core_add_CoreObj//�������� � ������ ���� ������ ���� + �����
        
        POP {R12,PC} //������� ���������
//=========�������������� ������������ ��������� ������ �� R0======       
svc_sw_thread_to_R0: //#32
    _save_op 75
    
    PUSH {LR}
//��������� ������� � ������������ �����
    //LDR R1, =pctcon //dbg
    //LDR R1,[R1]
    //LDR R1,[R1,#offs_id] //DBG
    CMP R0,#0
    BNE.N svc_sw_t_sw
    
errloop1: B errloop1 //R0==0 !

svc_sw_t_sw:    
    BL sub_switch_context_to_R0_ 
    
    POP {PC}
//==============================================================================
svc_scan_tasklist: //#33 �������������� ������ �����
    PUSH {LR}
    BL core_scan_tasklist
    POP {PC}
//==============================================================================
        END
//==================@ MW dem1305 2017====dem1305@yandex.ru===========EOF