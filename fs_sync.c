/**
  Laser project by dem1305

  @2017-2018
  
  функции синхронизации файловой системы
*/
//----------------------------------------------------------------------------//
#include "ff.h"

#if _FS_REENTRANT

#incude "core.h"

_SYNC_t* syncob[_VOLUMES];
//----------------------------------------------------------------------------//
int ff_cre_syncobj (BYTE vol, _SYNC_t* sobj)	/* Create a sync object */
{
  if(vol>_VOLUMES-1)while(1);//ERROR DBG 
  
  *sobj=syncob[vol]=(_SYNC_t)CreateMutex(0,FALSE,0);
  
  return 1;
}
//------------------------------------------------------------------------------
int ff_req_grant (_SYNC_t sobj)				/* Lock sync object */
{
  if(WaitForSingleObject(sobj,_FS_TIMEOUT)==WAIT_TIMEOUT)return 0;
  else return 1;
}
//------------------------------------------------------------------------------
void ff_rel_grant (_SYNC_t sobj);			/* Unlock sync object */
{
  BOOL r=ReleaseMutex( sobj );
  if(!r)while(1);//ERROR DBG 
}
//------------------------------------------------------------------------------
int ff_del_syncobj (_SYNC_t sobj)				/* Delete a sync object */
{
  BOOL r=CloseHandle((HANDLE)sobj);//освободить ресурсы дескриптора объекта
  if(!r)while(1);//ERROR DBG 
  return 1;
}
//----------------------------------------------------------------------------//
#endif
