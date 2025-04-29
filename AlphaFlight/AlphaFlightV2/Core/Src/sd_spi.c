/*
 * sd_diskio.c
 *
 *  Created on: Apr 29, 2025
 *      Author: benno
 */

#include "sd_spi.h"
#include "stdint.h"

static BYTE STA = STA_NOINIT;

DSTATUS SD_SPI_Initialize(BYTE pdrv){
	if(pdrv != 0) return STA_NOINIT;
	//init code
	STA &= ~STA_NOINIT;
	return STA;
}

DSTATUS SD_SPI_Status(BYTE pdrv){
	if(pdrv != 0) return STA_NOINIT;
	return STA;
}

DRESULT SD_SPI_ReadBlocks(BYTE pdrv ,BYTE *buffer, DWORD sector, UINT count){
	if(pdrv != 0) return RES_ERROR;
	return RES_OK;
}

DRESULT SD_SPI_WriteBlocks(BYTE pdrv, const BYTE *buffer, DWORD sector, UINT count){
	if(pdrv != 0) return RES_ERROR;
	return RES_OK;
}

DRESULT SD_SPI_Ioctl(BYTE pdrv, BYTE cmd, void *buff){
	if(pdrv != 0) return RES_ERROR;
	//do CMD stuff
	//if good
	return RES_OK;
	//else
	return RES_PARERR;
}

void SD_SPI_Process(void){

}
