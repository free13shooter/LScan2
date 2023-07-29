/**
  Laser project by dem1305

  @2017
  
  ���� ����������� ������ � �������� 
*/
//----------------------------------------------------------------------------//


#ifndef __CORE_DEF_H
#define __CORE_DEF_H
//----------------------------------------------------------------------------//
//���� ��������� ENABLE_MULTITHREADING=1 - �������� ������������� �����
#define ENABLE_USE_FPU                1 //������������� ������������
#define ENABLE_MULTITHREADING         1
#define DISABLE_MCU_BUFFERING_MODE    0 //��������� �����������(��������� �������)
#define CORE_MPU_PROTECTION           0 //1 //������������ MPU ��� ������ ������ ��������� ������ (DEBUG MODE)
  
#define MAX_MSP_STACK_TOP         0x200FFFFF//������� �������
#define DEFAULT_TSTACK_SIZE       0x1000      //4096 ���� ������������ �������


#define ENABLE_STACK_CHECK_DEBUG  0 //1 //���������� ��� ������� ��
#define CORE_DBG_OPERATION        0 //1 //���������� ��� ������� ��,������ ��������� ��������
#define CORE_DBG_LEN              8
//----------------------------------------------------------------------------//
/*
����������� ����������� ����������.
���������� ��� ������������ ���������� ������� ������� ���������� ��� ���������� 
���������� ������ ���������� � ������������� ������� EX_RETURN .
*/
#define CORE_NVIC_PriorityGroup         NVIC_PriorityGroup_2    // !!! ��� ������ �������� ���� PRIGROUP ��� OR-�������� ���� 8-10 SCB->AIRCR [0-3] ���������:[0-3]������������
#define CORE_IT_PRIO_GROUP              3                       //��������� ������ ����������
//----------------------------------------------------------------------------//
/*
������������ ������� ���������� �� ��������� TSV_TIMx_CYCLES ������ 
������� , ���������� �������� ������� � ���������� TSV_TIME_MS :
TSV_TIMx_CYCLES * TSV_TIME_MS = ����� ������� ������ ��������� 
*/
 
#define TSV_TIMx_CYCLES       5           //���������� ���������� ������� ��� ������������

#define TSV_TIME_MS           1           //�������� ������� ������������ ����������
//������ ���������� ����,��������� ������
#define TTIMER_IT_PRIO        CORE_IT_PRIO_GROUP  //��������� ���������� ������� ��������
#define TSVCALL_IT_PRIO       CORE_IT_PRIO_GROUP  //��������� ���������� SVCall
#define TPENDSV_IT_PRIO       CORE_IT_PRIO_GROUP  //��������� ���������� PendSV (����� ������)

//��������� ������ ����������� � ������ ��� ������������� �����������
#define TTIMER_IT_PRIO_SUB        1      //������������ ���������� ������� ��������
#define TSVCALL_IT_PRIO_SUB       0      //������������ ���������� SVCall
#define TPENDSV_IT_PRIO_SUB       2      //������������ ���������� PendSV (����� ������)

//----------------------------------------------------------------------------//
// MODE MASK
#define TPRIV_BIT_Pos       0
#define TPRIV_Msk           (1UL << TPRIV_BIT_Pos)

#define TSTACK_MODE_BIT_Pos 1
#define TSTACK_MODE_Msk     (1UL << TSTACK_MODE_BIT_Pos)

#define TFPU_MODE_BIT_Pos   2
#define TFPU_MODE_Msk       (1UL << TFPU_MODE_BIT_Pos)

#define TPRIO_BIT_Pos       3
#define TPRIO_Msk           (31UL << TPRIO_BIT_Pos)

#define TCRITSECT_BIT_Pos   8
#define TCRITSECT_Msk       (1UL << TCRITSECT_BIT_Pos)

#define TSUSPENDED_BIT_Pos  9
#define TSUSPENDED_Msk      (1UL << TSUSPENDED_BIT_Pos)

#define TSTACKSIZE_BIT_Pos  10
#define TSTACKSIZE_Msk      (1UL << TSTACKSIZE_BIT_Pos)

//     ************
#define TMMRAM_CTX_BIT_Pos  11   //   0x800 == 2048==�� ������������ ������ CCM_RAM ��� ���������
#define TMMRAM_CTX_Msk      (1UL << TMMRAM_CTX_BIT_Pos)

#define TMMRAM_STACK_BIT_Pos 12   //   0x1000 == 4096==�� ������������ ������ CCM_RAM ��� �����
#define TMMRAM_STACK_Msk     (1UL << TMMRAM_STACK_BIT_Pos)
//     ************

//----------������----------
#define TM_PRIV             0               //�����������������
#define TM_NPRIV            TPRIV_Msk       //�������������������

#define TM_MSP              0               //������������ MSP
#define TM_PSP              TSTACK_MODE_Msk //������������ PSP

/*
��������� �������� FPU � ����� PSP ������ (1=���)
CONTROL register bit 2 = FPCA: Indicates whether floating-point context currently active:
0: No floating-point context active
1: Floating-point context active.
*/
#define TM_NFPU             0
#define TM_FPU              TFPU_MODE_Msk

#define TM_CREATE_SUSPENDED TSUSPENDED_Msk  //�������������
#define TM_SET_STACK_SIZE   TSTACKSIZE_Msk  //��� �������� ������ ������ ������ �����
/* ======================================================================
      CONTROL register
      ������ ������ � ����������������� ������
      ������������ ������� MSR, MRS .
      ����� �����:

#define CTRL_PRIV    0x00000000 //��� 0 ����������������� ����� Thread Mode
#define CTRL_NPRIV   0x00000001 //��� 0 �� ����������������� ����� Thread Mode

#define CTRL_MSPSEL  0x00000000 //��� 1 ����� ����� ��� Handler Mode (0: MSP, 1: PSP)
#define CTRL_PSPSEL  0x00000002 //��� 1 ����� ����� ��� Handler Mode (0: MSP, 1: PSP)

#define CTRL_FPSP    0x00000004 //��������� �� �������� FPU � �����
*/
//EXC_RETURN
#define EXC_RET_USE_BITMSK  0x1F       //������������� ����

#define EXC_RET_BASE        0xFFFFFFE1

#define EXC_RET_HALDLER_MSP 0xFFFFFFF1 //������� � ����� ����������� c �������������� MSP[0001]
#define EXC_RET_THREAD_MSP  0xFFFFFFF9 //������� � ����� ������ c �������������� MSP[1001]
#define EXC_RET_THREAD_PSP  0xFFFFFFFB //������� � ����� ������ c �������������� PSP[1011]
//-----������� �������� ��������------
#define EXC_RET_MSP     0   //������� � �������������� ����� MSP
#define EXC_RET_PSP     4   //������� � �������������� ����� PSP

#define EXC_RET_HALDLER 0   //������� � ����� �����������
#define EXC_RET_THREAD  8   //������� � ����� ������
//�������� ���� ������������� FPU ��� �������� (���� ��� ��,��� � ������)
#define EXC_RET_NOFPU   16
#define EXC_RET_USEFPU  0


//----------------------------------------------------------------------------//
#define TSV_TIMER             TIM2                  //������ ������������
#define TSV_TIMER_BASE_ADR    0x40000000            //TIM2 BASE

#define TIMx_SR_OFFSET        0x10                  //TIM SR register offset
#define TIMx_DIER_OFFSET      0x0C                  //TIM DIER register offset

#define NVIC_BASE_ADR         0xE000E100  //!< NVIC Base Address                  */

#define TSV_TIM_NVIC_ISER_OFS 0      //�������� �������� ������.���������� �������
#define TSV_TIM_NVIC_ICER_OFS 0x080  //�������� �������� ������� ���������� �������
#define TSV_TIM_NVIC_BIT      29     //��� ��� ����������/������� ����������= 1<<(irq&1F)
//#define TSV_TIM_NVIC_BIT_VAL  (1UL<<TSV_TIM_NVIC_BIT)     

#define TSV_TIMER_RCC_APBCmd  RCC_APB1PeriphClockCmd
#define TSV_TIMER_APBPeriph   RCC_APB1Periph_TIM2
#define TSV_TIMER_PSC_1MS     ((uint16_t)2)
#define TSV_TIMER_ARR_1MS     ((uint16_t)42000)

#define TSV_TIMER_IRQ         TIM2_IRQn
#define TSV_TIMER_IRQ_HNDLR   TIM2_IRQHandler
//----------------------------------------------------------------------------//
//-------�������� � ����� (PSR PC LR R12 R3 R2 R1 R0) -------
#define st_PSR  28    
#define st_PC   24
#define st_LR   20
#define st_R12  16
#define st_R3   12
#define st_R2   8
#define st_R1   4
#define st_R0   0
//----------------------------------------------------------------------------//
#define _OCORE_SIZE   16    //������ ��������� ������� ����,� ������ (threads.h)
//--------thread struct offset vals--------
//--------��������� coreobj------->>
#define offs_ty       0     //16 ��� ���
#define offs_flags    2     //16 ��� �����
#define offs_sync     4     //������ ������ ������������� (���������������)
#define offs_prev     8     //���������� ��������
#define offs_next     12    //��������� ��������
//--------��������� coreobj-------<<
#define offs_mode     (0+_OCORE_SIZE)    //����� �������
#define offs_id       (4+_OCORE_SIZE)    //������������� ������ (int,�������������)

#define offs_tstack_PSP   (8+_OCORE_SIZE)    //������� ������� �����
#define offs_tstaRAM_PSP  (12+_OCORE_SIZE)   //������� �����
#define offs_tstsize_PSP  (16+_OCORE_SIZE)   //������ �����
//���������
#define offs_rsv0         (20+_OCORE_SIZE)   
#define offs_rsv1         (24+_OCORE_SIZE)
#define offs_rsv2         (28+_OCORE_SIZE)

#define offs_wait_ms      (32+_OCORE_SIZE)    //����� ����� INFINITE=(ULONG)-1 -������ ���

#define offs_sync_size    (36+_OCORE_SIZE)    //���������� �������� �������������

#define offs_R4_R11_regs  (40+_OCORE_SIZE)    //��������� �������� ����������
//----------------------------------------------------------------------------//
//���� �������� ����
#define TY_THREAD         0
#define TY_SEMAPHORE      1
#define TY_MUTEX          2
#define TY_EVENT          3
#define TY_SYSTIMER       4
#define TY_TASK           5

#define TY_MAX            TY_TASK

#define TY_ANY            0xFFFF //����� ���
//----------------------------------------------------------------------------//
// CORE OBJECT FLAGS :  ����� �������� ���� (16 ���)
#define CF_PASSIVE        0
#define CF_SIGNAL         1

#define CF_WAIT_ALL       2
#define CF_WAIT_MULTIPLE  4     

#define CF_AUTORESET      8      //������������� � event

#define CF_PLAIN          0      //��� �������� ���������� �������
#define CF_RECURSIVE      16
#define CF_TIMED          32

#define CF_CAPTSTATE      0x2000 //� �������� �������
#define CF_ABANDONED      0x4000 //������ ������ ����������� ��� ������� (����� ���������)
#define CF_TIMEOUT        0x8000 //�������
//----------------------------------------------------------------------------//

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address  */
#define ITM_BASE            (0xE0000000UL)                            /*!< ITM Base Address                   */
#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address            */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address               */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                  */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address  */

//SCS_BASE:
#define ICTR_OFFSET         0x004     /*!< Offset: 0x004 (R/ )  uint32_t Interrupt Controller Type Register     */
#define ACTLR_OFFSET        0x008     /*!< Offset: 0x008 (R/W) uint32_t Auxiliary Control Register              */
//-------------------------------
//1=������ ����������� ������ ��� ����������� ������ �� ��������� (����������� ������� ����)
#define SCnSCB_ACTLR_DISDEFWBUF_BitPos         1                                   /*!< ACTLR: DISDEFWBUF Position */
#define SCnSCB_ACTLR_DISDEFWBUF_MskVal        (1UL << SCnSCB_ACTLR_DISDEFWBUF_BitPos) /*!< ACTLR: DISDEFWBUF Mask */
//1=������ ���������� ��������������� ����������� ������/������ ��������� (LDM/STM)
#define SCnSCB_ACTLR_DISMCYCINT_BitPos         0                                   /*!< ACTLR: DISMCYCINT Position */
#define SCnSCB_ACTLR_DISMCYCINT_MskVal        (1UL << SCnSCB_ACTLR_DISMCYCINT_BitPos) /*!< ACTLR: DISMCYCINT Mask */
//-------------------------------
//System handler priority register 
#define SHPR_OFFSET     0x18
#define SHPR2_OFFSET    0x1C

//System handler co ntrol an d state register (SHCSR) , Address offset: 0x24
#define SHCSR_OFFSET    0x24

//!< Offset: 0x014 (R/W)  Configuration Control Register  
#define SCB_CCR_REG     0xE000ED14 //32 bit (2 WORD)

//0-������������ ����� 4 ����, 1-������������ ����� 8 ���� , ��� 9 PSR - ����� ������������
#define STKALIGN_VAL    0x200      //������� ��� � SBC->CCR   

#define SCB_SHCSR_SVCALLPENDED_BitPos         15                                             //< SCB SHCSR: SVCALLPENDED Position 
#define SCB_SHCSR_SVCALLPENDED_MskVal         (1UL << SCB_SHCSR_SVCALLPENDED_BitPos)         //< SCB SHCSR: SVCALLPENDED Mask 
//-------------------
#define SCB_ICSR_OFFSET 0x004 //(R/W)  Interrupt Control and State Register

#define SCB_ICSR_PENDSVSET_BitPos             28                                             /*!< SCB ICSR: PENDSVSET Position */
#define SCB_ICSR_PENDSVSET_MskVal             (1UL << SCB_ICSR_PENDSVSET_BitPos)                /*!< SCB ICSR: PENDSVSET Mask */

#define SCB_ICSR_PENDSVCLR_BitPos             27                                             /*!< SCB ICSR: PENDSVCLR Position */
#define SCB_ICSR_PENDSVCLR_MskVal             (1UL << SCB_ICSR_PENDSVCLR_BitPos)     
//----------------------------------------------------------------------------//
#define ERR_EXC_RET         1

#define ERR_STACK_LIM       6   //��������� ������� ����� PSP ���� ���������� ������� RAM
#define ERR_UP_STACK_LIM    2   //��������� ������� ����� PSP ���� ���������� ������� RAM
#define ERR_LO_STACK_LIM    4   //��������� ������� ����� PSP ���� ���������� ������� RAM
#define ERR_ZERO_PTR        8   //������� ��������� !
#define ERR_NOT_FOUND       16  //�� ������� !
#define ERR_OUT_OF_MEMORY   32  //��� ������
#define ERR_IT_STATE        64  //� ��������� ����������
//----------------------------------------------------------------------------//
#ifndef INFINITE
//��� ����������� ������������� � �������� 32 ���
#define INFINITE  (0xFFFFFFFFUL) //((ULONG)-1) 4 �����
#endif
//----------------------------------------------------------------------------//
#define NVIC_INT_CTRL_CONST 0xE000ED04 
//----------------------------------------------------------------------------//
#endif//__CORE_DEF_H