/**
  Laser project by dem1305

  @2017-2018
  
  функции синхронизации файловой системы
*/
//----------------------------------------------------------------------------//
#include "ff.h"



#if _FS_REENTRANT

//----------------------------------------------------------------------------//
#if USE_FAST_HARD_MUTEX==1
//мьютекс эксклюзивного доступа 
volatile unsigned long _mflock[_VOLUMES];//флаги блокировки

  #define _entry_sync_lock_(_mut_) ThreadSafeLockUnlock32Timed((unsigned long*)(_mut_),\
                                          true,\
                                          _FS_TIMEOUT)

  #define _leave_sync_lock_(_mut_) ThreadSafeLockUnlock32Timed((unsigned long*)(_mut_),\
                                          false,\
                                          _FS_TIMEOUT)

#else
  static HANDLE syncob[_VOLUMES];
#endif

//----------------------------------------------------------------------------//
int ff_cre_syncobj (BYTE vol, _SYNC_t* sobj)	/* Create a sync object */
{
 if(vol>=_VOLUMES)while(1);//ERROR DBG 
 
#if USE_FAST_HARD_MUTEX==1
  
  Atom32Set((unsigned long*)&_mflock[vol],0);
    
  *sobj=(unsigned long*)&_mflock[vol];
  
#else
  
  *sobj=syncob[vol]=(_SYNC_t)CreateMutex(0,FALSE,0);
  
  if(syncob[vol]==0)while(1);//DBG  
#endif  
  
  return 1;
}
//------------------------------------------------------------------------------
int ff_req_grant (_SYNC_t sobj)				/* Lock sync object */
{
#if USE_FAST_HARD_MUTEX==1
  if(_entry_sync_lock_(sobj)==WAIT_TIMEOUT) return 0;
      else return 1;
  
#else  
  if(WaitForSingleObject(sobj,_FS_TIMEOUT)==WAIT_TIMEOUT)
        return 0;
      else return 1;
#endif
  
}
//------------------------------------------------------------------------------
void ff_rel_grant (_SYNC_t sobj)			/* Unlock sync object */
{
#if USE_FAST_HARD_MUTEX==1
  if(_leave_sync_lock_(sobj)==WAIT_TIMEOUT) return ;//while(1);
#else
  if(!ReleaseMutex( sobj ))
        while(1); 
#endif  
}
//------------------------------------------------------------------------------
int ff_del_syncobj (_SYNC_t sobj)				/* Delete a sync object */
{
  for(uint8_t i=0;i<_VOLUMES;i++)
  {
    
#if USE_FAST_HARD_MUTEX==1
    if(((unsigned long*)&_mflock[i])==sobj)
    {
      Atom32Set((unsigned long*)&_mflock[i],0);
      return 1;
    }
#else
    if(syncob[i]==sobj)
    {
      CloseHandle((HANDLE)sobj);syncob[i]=0;
      return 1;
    }
#endif  
  }
  while(1);//ERROR DBG 
  
  //mfree(sobj);
  //return 1;
}

#endif
//----------------------------------------------------------------------------//
#if _USE_LFN==3

void* ff_memalloc (UINT msize)			/* Allocate memory block */
{
#ifdef _USE_LFN_MEM_CCM_DATARAM 
  void* r=mmmalloc(msize);
#else 
  void* r=ccmalloc(msize);
#endif
  if(!r)while(1);//ERROR DBG 
  return r;
}

void ff_memfree (void* mblock)			/* Free memory block */
{
  mfree(mblock);
}

#endif
//----------------------------------------------------------------------------//

