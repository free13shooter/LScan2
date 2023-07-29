/**
  ******************************************************************************
  * @file    CCM_RAM.h 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    17-Nov-2016
  * @note    ����������� ������������� 64 �� ������ Core-Couped-Memory.
  *          128 �� �������� ������ ������������ � �������� ������������ + ����������.
  * @note    CCM_RAM �� ����� �������������� ��� ������ � ����������(DMA,...),�� 
  * @note    �������� ������������� �� ��� ������������ ������
  ******************************************************************************
  */ 

//----------------------------------------------------------------------------//
/*
����������� ������������ ��������� � ������������ ������ malloc � free.
����� ������ ���������� � ���� ���������� ������ � ����� ��������(������������)/
�����������(��������������)������������� ��� ��������� � �������� ������.
� �������� ������ CCM �� ������������ � �������� � �������� �� ST,�� 64 �� ��� 
STM32F407 �� ������ ������� ����������� ������ 192 ��-��� ����� ���� ������,
������� ����� ������������ ���� ������� ������ ���,��� �� ��������� �������
��������� � ������� ������� DMA, �������� ������ ��������� ���� ��� ����.

�������� ������ ������������ ����������� ������� ����� ������ �������� ��,
���� ���� ������� mm_OS_atomic()
*/
//----------------------------------------------------------------------------//
#ifndef __CCM_MM_RAM_H
#define __CCM_MM_RAM_H

//----------------------------------------------------------------------------//
#include "stm32f4xx.h"

#include "LScan_types.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "core.h"
//----------------------------------------------------------------------------//
// CC MEMORY (64 KB)
/*
������ CCRAM ������������ �������� ����� MSP + 0x100 ���� ��� ���������� ����.
��� ������������ ����� �������������� ���������� ���������� ���������� �
����� ����� ���� 0x10000000 - ������ CORE COUPED MEMORY,��� ��������� �������.
����� ����� ��   0x10000100 �������� ������������ � ����������� ������� ����������.
*/
#define CCM_MSTACK_SIZE    0x1000 //MSP main stack
#define CCM_RAM_START      (CCMDATARAM_BASE+CCM_MSTACK_SIZE+0x100)   //������ ������� ������(��.datasheet)
#define CCM_RAM_END        (CCMDATARAM_BASE+0x0000FFFF)   //����� ������� ������(��.datasheet)
#define CCM_RAM_SIZE       (CCM_RAM_END-CCM_RAM_START)

#define CCHEAP             ((void*)CCM_RAM_START)
#define CCHEAP_SIZE        ((size_t)CCM_RAM_SIZE)
#define CCHEAP_END         ((void*)CCM_RAM_END) 
// MAIN MEMORY (128 KB-��������� ����������-����� ��� ������ ������ �������)
#define MMHEAP_SIZE        (0x00013000-0x100)
//----------------------------------------------------------------------------//
#pragma pack(1)
typedef struct {
  void* p;        //previous - ��������� �� ���������� ����
  void* n;        //next - ��������� �� ��������� ����
  uint32_t  b;    //busy - ���� �����? 32 ���� ��� ������������
}MBlock,*PMBlock;
#pragma pack()
//----------------------------------------------------------------------------//
#define msafe_free(mem)    if(mem){mfree(mem);(mem)=0;}
//----------------------------------------------------------------------------//
extern UINT    MM_RAM_START  ;//������ ������� ������(��.datasheet)
extern UINT    MM_RAM_END    ;//����� �������
extern UINT    MM_RAM_SIZE   ;//������ �������

extern void*   MMHEAP        ;//������ ������ ����
extern void*   MMHEAP_END    ;//����� ����

//��� ����� ������ CCRAM?
inline bool is_mtype_CCRAM(MBlock* b)
{
  return ((uint32_t)b>=CCM_RAM_START && (uint32_t)b<=CCM_RAM_END-sizeof(MBlock));
}

//��� ����� ������ MMRAM?
inline bool is_mtype_MMRAM(MBlock* b)
{
  return ((uint32_t)b>=MM_RAM_START && (uint32_t)b<=MM_RAM_END-sizeof(MBlock));
}

//��������� �� ���� ������ ��RAM|MMRAM
inline bool is_mtype_equal(MBlock* a,MBlock* b)
{
  bool a_ccram=((uint32_t)a>=CCM_RAM_START && (uint32_t)a<=CCM_RAM_END-sizeof(MBlock));
  bool b_ccram=((uint32_t)b>=CCM_RAM_START && (uint32_t)b<=CCM_RAM_END-sizeof(MBlock));
  return (a_ccram==b_ccram);
}
//----------------------------------------------------------------------------//
void  initheaps();//���������������� ���� CCM + MM
//----------------------------------------------------------------------------//
void* ccmalloc(size_t s);//��������� ������ �� ���� 
void* ccrealloc(void* m, size_t s);//����������������� ������ 

size_t ccgetsize(void* m);//�������� ������ ������ ����������� �����(0 � ������ ������)
size_t ccmaxmem();//�������� ����������� ��������� ������ ������

void* ccpush(void* pobj, size_t objsize);//���������� ������ � ���� CCM_RAM
void* mempush(void* pobj, size_t objsize);//���������� ������ � ����� ���� 
bool mempop(void* memoryRef,void* dest_pobj, size_t objsize);//������� ������ � �������������

size_t ccget_memory_left();
//----------------------------------------------------------------------------//
void* mmmalloc(size_t s);//��������� ������ �� ���� 
void* mmrealloc(void* m, size_t s);//����������������� ������ 
void  mfree(void* m);//������������ ������
#define ccfree mfree

size_t mmgetsize(void* m);//�������� ������ ������ ����������� �����(0 � ������ ������)
size_t mmmaxmem();//�������� ����������� ��������� ������ ������

void* mmpush(void* pobj, size_t objsize);//���������� ������ � ���� CCM_RAM
//void* mempush(void* pobj, size_t objsize);//���������� ������ � ����� ���� 
//bool mempop(void* memoryRef,void* dest_pobj, size_t objsize);//������� ������ � �������������

size_t mmget_memory_left();
//----------------------------------------------------------------------------//
#endif /* __CCM_MM_RAM_H */
/******************* (C) COPYRIGHT 2016 dem1305 *****END OF FILE****/
