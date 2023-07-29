/**
  ******************************************************************************
  * @file    CCM_RAM.h 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    17-Nov-2016
  * @note    возможность использования 64 КБ памяти Core-Couped-Memory.
  *          128 КБ основной памяти используется в качестве динамической + глобальная.
  * @note    CCM_RAM не может использоваться для обмена с периферией(DMA,...),но 
  * @note    возможно использование ее для динамической памяти
  ******************************************************************************
  */ 

//----------------------------------------------------------------------------//
/*
реализовано динамическое выделение и освобождение памяти malloc и free.
блоки памяти существуют в виде связанного списка и могут делиться(фрагментация)/
соединяться(дефрагментация)автоматически при обращении к операции памяти.
В основном память CCM не используется в проектах и примерах от ST,но 64 КБ для 
STM32F407 из общего размера оперативной памяти 192 КБ-это треть всей памяти,
поэтому можно использовать этот участок памяти там,где не требуется участие
периферии и прямого доступа DMA, например вторая системная куча или стек.

менеджер памяти обеспечивает атомарность доступа путем вызова сервисов ОС,
если была вызвана mm_OS_atomic()
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
начало CCRAM определяется размером стека MSP + 0x100 байт под переменные ядра.
при переполнении стека гарантированно произойдет разрушение переменных и
выход стека ниже 0x10000000 - начала CORE COUPED MEMORY,что облегчает отладку.
выход стека на   0x10000100 означает переполнение и пересекание области переменных.
*/
#define CCM_MSTACK_SIZE    0x1000 //MSP main stack
#define CCM_RAM_START      (CCMDATARAM_BASE+CCM_MSTACK_SIZE+0x100)   //начало области памяти(см.datasheet)
#define CCM_RAM_END        (CCMDATARAM_BASE+0x0000FFFF)   //конец области памяти(см.datasheet)
#define CCM_RAM_SIZE       (CCM_RAM_END-CCM_RAM_START)

#define CCHEAP             ((void*)CCM_RAM_START)
#define CCHEAP_SIZE        ((size_t)CCM_RAM_SIZE)
#define CCHEAP_END         ((void*)CCM_RAM_END) 
// MAIN MEMORY (128 KB-глобаьные переменные-место для стеков других потоков)
#define MMHEAP_SIZE        (0x00013000-0x100)
//----------------------------------------------------------------------------//
#pragma pack(1)
typedef struct {
  void* p;        //previous - указатель на предыдущий блок
  void* n;        //next - указатель на следующий блок
  uint32_t  b;    //busy - блок занят? 32 бита для выравнивания
}MBlock,*PMBlock;
#pragma pack()
//----------------------------------------------------------------------------//
#define msafe_free(mem)    if(mem){mfree(mem);(mem)=0;}
//----------------------------------------------------------------------------//
extern UINT    MM_RAM_START  ;//начало области памяти(см.datasheet)
extern UINT    MM_RAM_END    ;//конец области
extern UINT    MM_RAM_SIZE   ;//размер области

extern void*   MMHEAP        ;//начало памяти кучи
extern void*   MMHEAP_END    ;//конец кучи

//тип блока памяти CCRAM?
inline bool is_mtype_CCRAM(MBlock* b)
{
  return ((uint32_t)b>=CCM_RAM_START && (uint32_t)b<=CCM_RAM_END-sizeof(MBlock));
}

//тип блока памяти MMRAM?
inline bool is_mtype_MMRAM(MBlock* b)
{
  return ((uint32_t)b>=MM_RAM_START && (uint32_t)b<=MM_RAM_END-sizeof(MBlock));
}

//совпадают ли типы блоков ССRAM|MMRAM
inline bool is_mtype_equal(MBlock* a,MBlock* b)
{
  bool a_ccram=((uint32_t)a>=CCM_RAM_START && (uint32_t)a<=CCM_RAM_END-sizeof(MBlock));
  bool b_ccram=((uint32_t)b>=CCM_RAM_START && (uint32_t)b<=CCM_RAM_END-sizeof(MBlock));
  return (a_ccram==b_ccram);
}
//----------------------------------------------------------------------------//
void  initheaps();//инициализировать кучи CCM + MM
//----------------------------------------------------------------------------//
void* ccmalloc(size_t s);//выделение памяти из кучи 
void* ccrealloc(void* m, size_t s);//перераспределение памяти 

size_t ccgetsize(void* m);//получить размер памяти выделенного блока(0 в случае ошибки)
size_t ccmaxmem();//получить максимально доступный размер памяти

void* ccpush(void* pobj, size_t objsize);//разместить объект в куче CCM_RAM
void* mempush(void* pobj, size_t objsize);//разместить объект в общей куче 
bool mempop(void* memoryRef,void* dest_pobj, size_t objsize);//извлечь объект с освобождением

size_t ccget_memory_left();
//----------------------------------------------------------------------------//
void* mmmalloc(size_t s);//выделение памяти из кучи 
void* mmrealloc(void* m, size_t s);//перераспределение памяти 
void  mfree(void* m);//освобождение памяти
#define ccfree mfree

size_t mmgetsize(void* m);//получить размер памяти выделенного блока(0 в случае ошибки)
size_t mmmaxmem();//получить максимально доступный размер памяти

void* mmpush(void* pobj, size_t objsize);//разместить объект в куче CCM_RAM
//void* mempush(void* pobj, size_t objsize);//разместить объект в общей куче 
//bool mempop(void* memoryRef,void* dest_pobj, size_t objsize);//извлечь объект с освобождением

size_t mmget_memory_left();
//----------------------------------------------------------------------------//
#endif /* __CCM_MM_RAM_H */
/******************* (C) COPYRIGHT 2016 dem1305 *****END OF FILE****/
