/*
 * sd_diskio.c
 *
 *  Created on: Apr 29, 2025
 *      Author: benno
 */

#include "sd_spi.h"
#include "stdint.h"

static uint8_t STA = STA_NOINIT;

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

DRESULT SD_SPI_ReadBlocks(uint8_t *buffer, uint32_t sector, uint32_t count){
	return;
}

DRESULT SD_SPI_WriteBlocks(const uint8_t *buffer, uint32_t sector, uint32_t count){
	return;
}

DRESULT SD_SPI_Ioctl(BYTE cmd, void *buff){
	return;
}

void SD_SPI_Process(void){

}
