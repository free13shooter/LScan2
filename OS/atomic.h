/**
  Laser project by dem1305

  @2015
  атомарные операции 
*/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __ATOMIC_H
#define __ATOMIC_H


/* Includes ------------------------------------------------------------------*/
#include "core.h"
//----------------------------------------------------------------------------//
extern volatile HTHREAD  pctcon;//current thread   
extern volatile uint32_t SystemTicks; 
extern void call_set_sleep(HTHREAD hthrd, uint32_t timeMs);//приостановить поток hthrd на timeMs миллисекунд
//----------------------------------------------------------------------------//
//добавить и вернуть не измененное
inline unsigned long Atom32AddRetNoCh(unsigned long* ptr, unsigned long value)
{
  unsigned long oldValue, newValue;
  do
  {
     oldValue = __LDREXW((unsigned long*)ptr);
     newValue = oldValue + value;
  }while(__STREXW(newValue, (unsigned long*)ptr));
  return oldValue;
}
//----------------------------------------------------------------------------//
//добавить и вернуть не измененное
inline unsigned short Atom16AddRetNoCh(unsigned short* ptr, unsigned short value)
{
   unsigned short oldValue, newValue;
   do
   {
      oldValue = __LDREXH(ptr);
      newValue = oldValue + value;
   }while(__STREXH(newValue, ptr));
   return oldValue;
}
//----------------------------------------------------------------------------//
//добавить и вернуть не измененное
inline unsigned char Atom8AddRetNoCh(unsigned char* ptr, unsigned char value)
{
  unsigned char oldValue, newValue;
  do
  {
    oldValue = __LDREXB(ptr);
    newValue = oldValue + value;
  }while(__STREXB(newValue, ptr));
  return oldValue;
}
//----------------------------------------------------------------------------//
//добавить и вернуть измененное
inline unsigned long Atom32AddRetCh(unsigned long * ptr, unsigned long value)
{
  unsigned long oldValue, newValue;
  do
  {
    oldValue = __LDREXW((unsigned long*)ptr);
    newValue = oldValue + value;
  }while(__STREXW(newValue, (unsigned long*)ptr));
  return newValue;
}
//----------------------------------------------------------------------------//
//добавить и вернуть измененное
inline unsigned short Atom16AddRetCh(unsigned short* ptr, unsigned short value)
{
 unsigned short oldValue, newValue;
 do
 {
   oldValue = __LDREXH(ptr);
   newValue = oldValue + value;
 }while(__STREXH(newValue, ptr));
 return newValue;       
}
//----------------------------------------------------------------------------//
//добавить и вернуть измененное
inline unsigned char Atom8AddRetCh(unsigned char* ptr, unsigned char value)
{
  unsigned char oldValue, newValue;
  do
  {
    oldValue = __LDREXB(ptr);
    newValue = oldValue + value;
  }while(__STREXB(newValue, ptr));
  return newValue;
}
//----------------------------------------------------------------------------//
//сравнить,записать,вернуть удачно ли
inline unsigned char Atom32CompExch(unsigned long* ptr, 
                                    unsigned long oldValue, 
                                    unsigned long newValue)
{
  // эксклюзивно читаем значение переменной и сравниваем со старым значением 
  // пытаемся эксклюзивно записать в переменную новое значение
  if(__LDREXW((unsigned long*)ptr) == oldValue)
  {
    return (unsigned char)__STREXW(newValue, (unsigned long*)ptr) == 0;
  }
  // кто-то изменил ячейку до нас 
  __CLREX();//сбросить бит экс.доступа
  return (unsigned char)0;
}
//----------------------------------------------------------------------------//
//сравнить,записать,вернуть удачно ли
inline unsigned char Atom16CompExch(unsigned short* ptr,
                                    unsigned short oldValue,
                                    unsigned short newValue)
{
  // эксклюзивно читаем значение переменной и сравниваем со старым значением 
  // пытаемся эксклюзивно записать в переменную новое значение
  if(__LDREXH(ptr) == oldValue) {return __STREXH(newValue, ptr) == 0;}
  // кто-то изменил ячейку до нас 
  __CLREX();//сбросить бит экс.доступа
  return 0;
}
//----------------------------------------------------------------------------//
//сравнить,записать,вернуть удачно ли
inline unsigned char Atom8CompExch(unsigned char * ptr,
                                   unsigned char oldValue,
                                   unsigned char newValue)
{
  // эксклюзивно читаем значение переменной и сравниваем со старым значением 
  // пытаемся эксклюзивно записать в переменную новое значение
  if(__LDREXB(ptr) == oldValue) {return __STREXB(newValue, ptr) == 0;}
  // кто-то изменил ячейку до нас 
  __CLREX();//сбросить бит экс.доступа
  return 0;
}
//----------------------------------------------------------------------------//
//записать и вернуть новое значение
inline unsigned long Atom32Set(unsigned long* ptr,unsigned long newValue)
{
  do
  {
    __LDREXW((unsigned long*)ptr);
  }
  while(__STREXW(newValue, (unsigned long*)ptr));
  return newValue;
}

inline unsigned long Atom32Get(unsigned long* ptr)
{
  unsigned long v=__LDREXW(ptr);__CLREX();
  return v;
}
//----------------------------------------------------------------------------//
//записать и вернуть новое значение
inline unsigned short Atom16Set(unsigned short* ptr,unsigned short newValue)
{
  do
  {
    __LDREXH(ptr);
  }
  while(__STREXH(newValue, ptr));
  return newValue;
}

inline unsigned short Atom16Get(unsigned short* ptr)
{
  unsigned short v=__LDREXH(ptr);__CLREX();
  return v;
}
//----------------------------------------------------------------------------//
//записать и вернуть новое значение
inline unsigned char Atom8Set(unsigned char* ptr,unsigned char newValue)
{
  do
  {
    __LDREXB(ptr);
  }
  while(__STREXB(newValue, ptr));
  return newValue;
}

inline unsigned char Atom8Get(unsigned char* ptr)
{
  unsigned char v=__LDREXB(ptr);__CLREX();
  return v;
}
//----------------------------------------------------------------------------//
//управление блокировкой эксклюзивного доступа.32 бит флаг-значение блокировки.
inline void ThreadSafeLockUnlock32(unsigned long* lock_ptr,bool b_do_lock)
{
  unsigned long it=(__get_IPSR() & 0xFF);
    
  if(it)while(1);//IT!    
  unsigned long id=pctcon->id;
  unsigned long chval=b_do_lock?0:id;
  unsigned long sval=b_do_lock?id:0;
  unsigned long cid;
  
  do
  {
    while((cid=__LDREXW(lock_ptr))!=chval)//id==thread id ?
    {
      __CLREX();
      if(cid!=0 && cid!=id)call_set_sleep(0,0);//освободить квант для завершения другим потоком
    }
  }
  while( __STREXW(sval,lock_ptr) );
}

//управление блокировкой эксклюзивного доступа.32 бит флаг-значение блокировки.
inline void ThreadSafeLockUnlock32Sleep(unsigned long* lock_ptr,bool b_do_lock,uint32_t sleep_time)
{
  unsigned long it=(__get_IPSR() & 0xFF);
    
  if(it)while(1);//IT!    
  unsigned long id=pctcon->id;
  unsigned long chval=b_do_lock?0:id;
  unsigned long sval=b_do_lock?id:0;
  unsigned long cid;
  
  do
  {
    while((cid=__LDREXW(lock_ptr))!=chval)//id==thread id ?
    {
      __CLREX();
      if(cid!=0 && cid!=id)call_set_sleep(0,sleep_time);//освободить квант для завершения другим потоком
    }
  }
  while( __STREXW(sval,lock_ptr) );
}
//----------------------------------------------------------------------------//
//управление блокировкой эксклюзивного доступа.32 бит флаг-значение блокировки.
inline DWORD ThreadSafeLockUnlock32Timed(unsigned long* lock_ptr,
                                        bool b_do_lock,
                                        unsigned long timeout_ms)
{
  unsigned long it=(__get_IPSR() & 0xFF);
    
  if(it)while(1);//IT!   
  
  unsigned long tout=(unsigned long)SystemTicks+timeout_ms;  
  unsigned long id=pctcon->id;
  bool btout=false;
  unsigned long chval=b_do_lock?0:id;
  unsigned long sval=b_do_lock?id:0;
  unsigned long cid;
  
  do
  {
    while((cid=__LDREXW(lock_ptr))!=chval)//id==thread id ?
    {
      __CLREX();//id==thread id ?
      if( ( btout=(timeout_ms!=0 && (SystemTicks>=tout)) )==true )break;
      if(cid!=0 && cid!=id)call_set_sleep(0,0);//освободить квант для завершения другим потоком
    }
  }
  while( __STREXW(sval,lock_ptr) && ( ( btout=(timeout_ms!=0 && (SystemTicks>=tout)) )==false ) );
    
  if(btout)return WAIT_TIMEOUT;
  
  return 0;//success
}
//----------------------------------------------------------------------------//
//управление блокировкой эксклюзивного доступа.32 бит флаг-значение блокировки.
inline void ExclusiveLockUnlock32( unsigned long lock_value,
                                    unsigned long* lock_ptr,
                                    bool b_do_lock)
{
  unsigned long chval=b_do_lock?0:lock_value;
  unsigned long sval=b_do_lock?lock_value:0;
  
  do
  {
    while(__LDREXW(lock_ptr)!=chval)__CLREX();//id==thread id
  }
  while(__STREXW(sval,lock_ptr));
}
//----------------------------------------------------------------------------//
//управление блокировкой эксклюзивного доступа.8 бит флаг-значение блокировки.
inline void ExclusiveLockUnlock8(unsigned char lock_value,
                                  unsigned char* lock_ptr,
                                  bool b_do_lock)
{
  unsigned char chval=b_do_lock?0:lock_value;
  unsigned char sval=b_do_lock?lock_value:0;
  
  do
  {
    while(__LDREXB(lock_ptr)!=chval)__CLREX();
  }
  while(__STREXB(sval,lock_ptr));
}

#define ExclusiveLockUnlockBool(_lock_ptr,_b_do_lock) ExclusiveLockUnlock8(1,(_lock_ptr),(_b_do_lock))
//----------------------------------------------------------------------------//
#endif
/******************* (C) COPYRIGHT 2015 dem1305 *****END OF FILE****/
