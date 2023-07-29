/**
  ******************************************************************************
  * @file    SPI2_media.h 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    7-Nov-2016
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __SPI_CARD_H 
#define  __SPI_CARD_H 
/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_dma.h"
#include "ffconf.h"
#include "diskio.h"
#include "LScan.h" //systick 1 ms

/* Private typedef -----------------------------------------------------------*/

void SPI2_init(uint8_t isMasterMode);//1=master 0=slave
void disk_timerproc (void);

DWORD socket_is_write_protected(void);
DWORD socket_is_empty(void);

DSTATUS spi_disk_initialize (	BYTE drv/* Physical drive number (0) */);
DSTATUS spi_disk_status (BYTE drv/* Physical drive number (0) */);
DRESULT spi_disk_read (
	BYTE drv,			/* Physical drive number (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..255) */
);

#if _FS_READONLY == 0
DRESULT spi_disk_write (
	BYTE drv,			/* Physical drive number (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..255) */
);
#endif


DRESULT spi_disk_ioctl (
  BYTE drv,		/* Physical drive number (0) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
);

/* ---------------------------------------------------------------------------*/
//включение/выключение канала прерывания core_cm4.h

/* ---------------------------------------------------------------------------*/
#endif /* __SPI_CARD_H */
/******************* (C) COPYRIGHT 2016 dem1305 *****END OF FILE****/