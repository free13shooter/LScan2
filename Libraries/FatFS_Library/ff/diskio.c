/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "ffconf.h"
#include "sdcard.h"
#include "spi_card.h"

#define DISK_IO_OS_SYNC         1   //использовать ли примитивы синхронизации потоков ОС
#define NUM_SYNC_DRIVES         _VOLUMES   //количество объектов синхронизации равна кол-ву дисков
#define SYNC_WAIT_MS     INFINITE   //время ожидания объекта синхронизации
/* Definitions of physical drive number for each drive */
#define DEV_MMC0		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC1		1	/* Example: Map MMC/SD [SDIO interface]card to physical drive 1 */
#define DEV_USB		        2	/* Example: Map USB MSD to physical drive 2 */

#define SPI_MMC   DEV_MMC0 //используется как постоянный системный диск

#define ROM_disk_status spi_disk_status
DSTATUS USB_disk_status();
#define ROM_disk_initialize spi_disk_initialize
DSTATUS USB_disk_initialize();
#define ROM_disk_read spi_disk_read
DRESULT USB_disk_read(BYTE* buff, DWORD sector, UINT count);
#define ROM_disk_write spi_disk_write
DRESULT USB_disk_write(const BYTE* buff, DWORD sector, UINT count);
#define ROM_disk_ioctl spi_disk_ioctl
DRESULT USB_disk_ioctl(BYTE pdrv,BYTE cmd,void *buff);
/*-----------------------------------------------------------------------*/
#if DISK_IO_OS_SYNC==1

  #include "atomic.h"

  extern volatile HTHREAD pctcon;
  //мьютекс эксклюзивного доступа 
  volatile unsigned long ioflock[NUM_SYNC_DRIVES];//флаги блокировки
  
  #define _entry_sync_(_drive_) ThreadSafeLockUnlock32((unsigned long*)&ioflock[(_drive_)],true) 
  #define _leave_sync_(_drive_) ThreadSafeLockUnlock32((unsigned long*)&ioflock[(_drive_)],false)

#else
  #define _entry_sync_(_drive_)
  #define _leave_sync_(_drive_) 
#endif
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
  DSTATUS stat=0;//OK
  switch (pdrv) 
  {
    case DEV_MMC0 :
      	
      if (socket_is_empty())stat |= STA_NODISK;
  
      return stat;

    case DEV_MMC1 :
    
      if (SD_Detect() != SD_PRESENT)stat |= STA_NODISK;
    
      return stat;

    case DEV_USB :
      
      return USB_disk_status();

  }
  return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
  DSTATUS stat=STA_NOINIT;
  
  _entry_sync_(pdrv);

  switch (pdrv) 
  {
  case DEV_MMC0 :
    stat = ROM_disk_initialize(pdrv);
    break;
  case DEV_MMC1 :
    stat= (DSTATUS) SD_Init();//MMC_disk_initialize();
    break;
  case DEV_USB :
    stat= (DSTATUS)USB_disk_initialize();
  }
  
  _leave_sync_(pdrv);
  return stat;
}
/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
  UINT i;
  SDTransferState state;
  SD_Error SDstat=SD_OK;
  DRESULT dr=RES_PARERR;
  
  _entry_sync_(pdrv);
  
switch (pdrv) 
{
  case DEV_MMC0 :// translate the arguments here
    dr= ROM_disk_read(DEV_MMC0,buff, sector, count);
    break;

  case DEV_MMC1 :
    if ( SD_Detect( ) != SD_PRESENT) {dr= RES_NOTRDY;break;}
    if ( count == 0){dr= RES_PARERR;break;}
    #if defined (SD_DMA_MODE)
      SDstat=SD_ReadMultiBlocksFIXED ( buff, sector, 512, count );
      if(SDstat!=SD_OK){dr= (DRESULT)SDstat;break;}
      SDstat=SD_WaitReadOperation ( );
    #elif
      SDstat=SD_ReadBlock ( buff, sector, 512 );//(uint8_t *readbuff, uint64_t ReadAddr, uint16_t BlockSize)//POOLING MODE
    #endif
      if(SDstat!=SD_OK){dr= (DRESULT)SDstat;break;}
      //Антилок
      i=200000;
      state = SD_TRANSFER_BUSY;
      while ( --i )
      {
        state = SD_GetStatus ( );
        if ( state == SD_TRANSFER_OK ){dr=RES_OK;break;}
      } // while
  
     dr= (DRESULT)state;
     break;

  case DEV_USB :
    dr= USB_disk_read(buff, sector, count);
  }

  _leave_sync_(pdrv);
  return dr;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
  UINT i;
  SD_Error SDstat=SD_OK;
  SDTransferState state;
  DRESULT dr=RES_PARERR;
  
  _entry_sync_(pdrv);

  switch (pdrv) 
  {
  case DEV_MMC0 :
    dr= ROM_disk_write(DEV_MMC0,buff, sector, count);
    break;
  case DEV_MMC1 :
    if ( SD_Detect( ) != SD_PRESENT ){dr=RES_NOTRDY;break;}
    if ( count == 0){dr= RES_PARERR;break;}
      
    #if defined (SD_DMA_MODE)
      SDstat=SD_WriteMultiBlocksFIXED ( (uint8_t *)buff, sector, 512, count );
    if(SDstat!=SD_OK){dr=(DRESULT)SDstat;break;}
      SDstat=SD_WaitWriteOperation ( );
    #elif
      SDstat=SD_WriteBlock ( (uint8_t *)buff, sector, 512 );//POOLING MODE
    #endif
      if(SDstat!=SD_OK){dr=(DRESULT)SDstat;break;}
      //Антилок
    i=200000;
    while ( --i )
    {
      state = SD_GetStatus ( );
      if ( state == SD_TRANSFER_OK ){dr=RES_OK;break;}
    } // while
  
      
    break;
      
  case DEV_USB :
     dr= USB_disk_write(buff, sector, count);
  
  }

  _leave_sync_(pdrv);
  return dr;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
DRESULT dr=RES_PARERR;
_entry_sync_(pdrv);

switch (pdrv) 
{
case DEV_MMC0 :// Process of the command for the RAM drive
		
  dr= ROM_disk_ioctl(pdrv,cmd,buff);
  break;
case DEV_MMC1 :// Process of the command for the MMC/SD card
  dr= (DRESULT)MMC_disk_ioctl(cmd,buff);
  break;	
case DEV_USB :// Process of the command the USB drive
  dr=  USB_disk_ioctl(pdrv,cmd,buff);
}

_leave_sync_(pdrv);
return dr;
}

/*-----------------------------------------------------------------------*/

DSTATUS USB_disk_status(){return RES_NOTRDY ;}

DSTATUS USB_disk_initialize(){return RES_NOTRDY ;}

DRESULT USB_disk_read(BYTE* buff, DWORD sector, UINT count){return RES_NOTRDY ;}

DRESULT USB_disk_write(const BYTE* buff, DWORD sector, UINT count){return RES_NOTRDY ;}

DRESULT USB_disk_ioctl(BYTE pdrv,BYTE cmd,void *buff){return RES_NOTRDY ;}