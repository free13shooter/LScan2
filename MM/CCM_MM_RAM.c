/**
  ******************************************************************************
  * @file    sys_tasks.c 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    17-Nov-2016
  * @note    возможность использования 64 КБ памяти Core-Couped-Memory.
  ******************************************************************************
  */ 

//----------------------------------------------------------------------------//
#include "CCM_MM_RAM.h"
#include "core_cm4.h"
//----------------------------------------------------------------------------//
// MAIN MEMORY (128 KB-глобаьные переменные-место для стеков других потоков)
UINT    MM_RAM_START  ;//(SRAM1_BASE)                  //начало области памяти(см.datasheet)
UINT    MM_RAM_END    ;//(SRAM1_BASE+0x00012000-0x8)   //позволяет malloc
UINT    MM_RAM_SIZE   ;//(0x00012000-0x8);

void*   MMHEAP        ;
void*   MMHEAP_END    ;
//----------------------------------------------------------------------------//
#define addrToBlock(v)  ((MBlock*)((UINT)(v)-sizeof(MBlock))) //преобразование 
//----------------------------------------------------------------------------//
extern void dbg_error_led(uint32_t errormask_R0);

static MBlock* mdivb(MBlock* b,size_t s);//разделить блок
static MBlock* msplb(MBlock* b1,MBlock* b2);//склеить блоки
static inline size_t mbsz(MBlock* pb);//получить размер занятого блока (0 в случае ошибки)
static inline size_t mfullbsz(MBlock* pb);//получить полный занятого блока с размером структуры (0 в случае ошибки)

static inline void* ccmm_malloc(size_t s,bool MM);//выделение памяти из кучи
static inline void* ccmm_realloc(void* m, size_t s,bool MM);//перераспределение памяти 
static inline void  ccmm_free(void* m,bool MM);//освобождение памяти
static inline void* ccmm_push(void* pobj, size_t objsize,bool MM);//разместить объект в куче
static inline void* ccmm_mempush(void* pobj, size_t objsize,bool MM);//разместить объект в куче
static inline size_t ccmm_maxmem(bool MM);

static volatile size_t ccmemory_left=CCHEAP_SIZE; //CCM Memory
static volatile size_t mmmemory_left=0;           //Main Memory
//----------------------------------------------------------------------------//
//мьютекс эксклюзивного доступа 
extern volatile unsigned char _mm_flag_lock_; //флаг блокировки

#define _mmlock   _mmLockUnlock(true)
#define _mmunlock _mmLockUnlock(false)

inline void _mmLockUnlock(bool b_do_lock)
{
  unsigned char chval=b_do_lock?0:1;
  unsigned char sval=b_do_lock?1:0;
  
  do
  {
    while(__LDREXB((unsigned char*)&_mm_flag_lock_)!=chval)
    {
      __CLREX();//id==thread id
      if(!(__get_IPSR() & 0xFF))Sleep(0);
    }
  }
  while(__STREXB(sval,(unsigned char*)&_mm_flag_lock_));
}


//----------------------------------------------------------------------------//
void  initheaps()//инициализировать кучу
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CCMDATARAMEN, ENABLE);//clock
  ZeroMemory(CCHEAP,sizeof(MBlock));//to NULL
  ccmemory_left-=sizeof(MBlock);
  //-----------------------------
  MM_RAM_START=(UINT)malloc(mmmemory_left=MM_RAM_SIZE=MMHEAP_SIZE); 
  if(MM_RAM_START==0)while(1);//ERROR init heap Main Memory
  
  MMHEAP=(void*)MM_RAM_START;
  MM_RAM_END=MM_RAM_START+MM_RAM_SIZE;
  MMHEAP_END=(void*)MM_RAM_END;
  ZeroMemory(MMHEAP,sizeof(MBlock));//to NULL
  mmmemory_left-=sizeof(MBlock);
}
//----------------------------------------------------------------------------//
size_t ccget_ccmemory_left()
{
  return ccmemory_left;
}

size_t mmget_ccmemory_left()
{
  return mmmemory_left;
}
//----------------------------------------------------------------------------//
size_t mm_maxmem(){return ccmm_maxmem(true);}//получить максимально доступный размер памяти
size_t cc_maxmem(){return ccmm_maxmem(false);}//получить максимально доступный размер памяти
static inline size_t ccmm_maxmem(bool MM)
{
  _mmlock;
  MBlock* b=MM?MMHEAP:CCHEAP;
  UINT e=MM?MM_RAM_END:CCM_RAM_END;
  size_t s=0;
  
  do
  {
    size_t bs= mbsz(b);
    if(!b->b &&bs>s )s=bs;
    b=b->n;
  }
  while(b && (UINT)b<e);
  _mmunlock;
  return s;
}
//----------------------------------------------------------------------------//
static inline void* ccmm_malloc(size_t s,bool MM)//выделение памяти из кучи 
{
  _mmlock;
  if((MM?mmmemory_left:ccmemory_left)<s+sizeof(MBlock))
  {
    _mmunlock;
    dbg_error_led(ERR_OUT_OF_MEMORY);
    //while(1);
    return NULL;
  }
  
  MBlock* b=MM?MMHEAP:CCHEAP;
  //size_t sss=sizeof(MBlock);//==12
  do
  {
    if(!b->b)
    {
      size_t sz=mbsz(b);
      if(sz<s)continue;
      if(sz>s && sz>sizeof(MBlock)*2)
      {
        b=mdivb(b,s);//разделить блок
        if(!b) //CRIT !
        {
          dbg_error_led(ERR_NOT_FOUND);
          while(1);
        }
      }
      
      b->b=true;
      if(MM)mmmemory_left-=(sizeof(MBlock)+s);
        else ccmemory_left-=(sizeof(MBlock)+s);
      
      _mmunlock;
      return (void*)((UINT)b+(UINT)sizeof(MBlock));//начало области памяти 
    }
  }
  while((b=b->n)!=NULL);
  dbg_error_led(ERR_OUT_OF_MEMORY);//НЕТ ПАМЯТИ !
  while(1);
  _mmunlock;
  return NULL;
}
//----------------------------------------------------------------------------//
static inline void* ccmm_realloc(void* m, size_t s,bool MM){return NULL;}//перераспределение памяти 
//----------------------------------------------------------------------------//
static inline void  ccmm_free(void* m,bool MM)//освобождение памяти
{
  _mmlock;
 /* if((MM==false &&((uint32_t)m<CCM_RAM_START || (uint32_t)m>CCM_RAM_END))||
  (MM==true &&((uint32_t)m<MM_RAM_START || (uint32_t)m>MM_RAM_END)))
  {
    while(1);
  }*/
  
  MBlock* b;
  if(m==0){_mmunlock;return;}
  b=addrToBlock(m);
  if(b->b==false){_mmunlock;return;}//уже свободен
  
  b->b=false;
  
  if(MM)mmmemory_left+=mfullbsz(b);else ccmemory_left+=mfullbsz(b);
  
  //дефрагментация
  while(b && b->p && ((MBlock*)(b->p))->b==0 &&
        (is_mtype_equal(b,(MBlock*) b->p)))//склеить с предыдущими 
  {
    b=msplb((MBlock*)(b->p),b);
    if(!b)while(1);//critical error
  }
  
  while(b && b->n && ((MBlock*)(b->n))->b==0&&
        (is_mtype_equal(b,(MBlock*) b->n)))//склеить с последующими
  {
    b=msplb(b,(MBlock*)(b->n));
    if(!b)while(1);//critical error
  }
  _mmunlock;
}
//----------------------------------------------------------------------------//
void* mmpush(void* pobj, size_t objsize)//разместить объект в куче MM_RAM
{
  return ccmm_push(pobj,objsize,true);//разместить объект в куче
}

void* ccpush(void* pobj, size_t objsize)//разместить объект в куче CCM_RAM
{
  return ccmm_push(pobj,objsize,false);//разместить объект в куче
}

static inline void* ccmm_push(void* pobj, size_t objsize,bool MM)//разместить объект в куче
{
  if(objsize==0)return NULL;
  void* mem;
  mem=ccmm_malloc(objsize,MM);
  if(!mem)return NULL;
  memcpy(mem,pobj,objsize);//to memory
  return mem;
}

void* ccmempush(void* pobj, size_t objsize)//разместить объект в куче
{
  return ccmm_mempush(pobj,objsize,false);//разместить объект в куче
}
void* mmmempush(void* pobj, size_t objsize)//разместить объект в куче
{
  return ccmm_mempush(pobj,objsize,true);//разместить объект в куче
}

static inline void* ccmm_mempush(void* pobj, size_t objsize,bool MM)//разместить объект в куче
{
  if(objsize==0)return NULL;
  void* mem;
  mem=ccmm_malloc(objsize,MM);
  if(!mem)return NULL;
  memcpy(mem,pobj,objsize);//to memory
  return mem;
}
//----------------------------------------------------------------------------//
/*
извлечь объект с освобождением:
memoryRef-место хранения объекта
dest_pobj-место назначения извлекаемого объекта
objsize-размер объекта
выбор типа памяти определяется по принадлежности к памяти 
Core Couped SRAM(диапазон ССHEAP)/aliased Bit-banding SRAM(диапазон общей памяти)
*/
bool mempop(void* memoryRef,void* dest_pobj, size_t objsize)
{
  if(objsize==0)return false;
  memcpy(dest_pobj,memoryRef,objsize);
  mfree(memoryRef); 
  return true;
}
//----------------------------------------------------------------------------//
size_t mgetsize(void* m)//получить размер памяти выделенного блока(0 в случае ошибки)
{
  return mbsz(addrToBlock(m));
}
//----------------------------------------------------------------------------//
static inline size_t mbsz(MBlock* pb)//получить размер занятого блока без размера структуры (0 в случае ошибки)
{
  size_t sz;
  if(!pb->n)
  {
    sz= is_mtype_CCRAM(pb)?(UINT)CCHEAP_END - (UINT)pb:(UINT)MMHEAP_END - (UINT)pb;
  }
  else sz=(UINT)(pb->n)-(UINT)pb;
  if(sz<=sizeof(MBlock))return 0;
  return sz-sizeof(MBlock);
}

static inline size_t mfullbsz(MBlock* pb)//получить полный занятого блока с размером структуры (0 в случае ошибки)
{
  size_t sz;
  if(!pb->n)sz= is_mtype_CCRAM(pb)?(UINT)CCHEAP_END - (UINT)pb:(UINT)MMHEAP_END - (UINT)pb;
  else sz=(UINT)(pb->n)-(UINT)pb;
  if(sz<sizeof(MBlock))while(1);//ERROR ! размер не может быть меньше блока
  return sz;
}
//----------------------------------------------------------------------------//
static MBlock* mdivb(MBlock* b,size_t s)//разделить блок (только свободный)
{
 size_t sb=mfullbsz(b);
 
 if(b->b || sb<=sizeof(MBlock) || s>=sb-sizeof(MBlock))return NULL;
 if(sb<=sizeof(MBlock)*2)return b;//не разделять,слишком мал
 //выбор блока для дефрагментации
 size_t sp=0; 
 size_t sn=0; 
 //слева-меньший адрес:
 if(b->p && ((MBlock*)(b->p))->b==false)sp=mbsz(b->p);//слева свободен
 if(b->n && ((MBlock*)(b->n))->b==false)sn=mbsz(b->n);//справа свободен
 
 MBlock* nb;
  
 if(sp>sn)//слева свободно больше = новый справа ----|b|nb|---- 
 {
   //начало блока + полный размер блока - нужный размер - размер структуры
   nb=(MBlock*)((UINT)b+(UINT)sb-(UINT)s-sizeof(MBlock));
   nb->b=0;
   nb->n=b->n;
   b->n=nb;
   nb->p=b;
 }
 else //справа свободно больше = новый слева ----|nb|b|----
 {
   nb=b;
   //начало блока + размер структуры + нужный размер
   b=(MBlock*)((UINT)nb+sizeof(MBlock)+(UINT)s);
   b->n=nb->n;
   b->p=nb;
   b->b=0;
   nb->n=b;
 }
 
 if(is_mtype_CCRAM(b))ccmemory_left-=sizeof(MBlock);
  else mmmemory_left-=sizeof(MBlock);
 
 return nb;
}
//----------------------------------------------------------------------------//
static MBlock* msplb(MBlock* b1,MBlock* b2)//склеить блоки
{
 size_t sb=mbsz(b1)+mbsz(b2);
 
 //из разной памяти ?
 bool mt1=is_mtype_CCRAM(b1);
 bool mt2=is_mtype_CCRAM(b2);
 
 if(mt1!=mt2 || b1==b2 || sb==0 || b1->b || b2->b)//ERROR !!!
 {
   while(1);
 }
  
 MBlock* nb;
 
 if(b1<b2)
 {
   nb=(MBlock*)b1;
   nb->p=b1->p;
   nb->n=b2->n;
 }
 else
 {
   nb=(MBlock*)b2;
   nb->p=b2->p;
   nb->n=b1->n;
 }
 
 if(mt1)ccmemory_left+=sizeof(MBlock);else mmmemory_left+=sizeof(MBlock);
 
 return nb;
}
//----------------------------------------------------------------------------//
void* ccmalloc(size_t s)//выделение памяти из кучи 
{
  return (ccmm_malloc(s,0));
}
//----------------------------------------------------------------------------//
void* ccrealloc(void* m, size_t s)//перераспределение памяти 
{
  return (ccmm_realloc(m,s,0));
}
//----------------------------------------------------------------------------//
void* mmmalloc(size_t s)//выделение памяти из кучи 
{
  return (ccmm_malloc(s,1));
}
//----------------------------------------------------------------------------//
void* mmrealloc(void* m, size_t s)//перераспределение памяти 
{
  return (ccmm_realloc(m,s,1));
}
//----------------------------------------------------------------------------//
void  mfree(void* m)//освобождение памяти
{
  if((uint32_t)m>=CCM_RAM_START && (uint32_t)m<=CCM_RAM_END)
  {
    ccmm_free(m,0);
    return ;
  }
  else if((uint32_t)m>=MM_RAM_START && (uint32_t)m<=MM_RAM_END)
  {
    ccmm_free(m,1);
    return ;
  }
  
  while(1);//ERROR
}
//----------------------------------------------------------------------------//



//----------------------------------------------------------------------------//
