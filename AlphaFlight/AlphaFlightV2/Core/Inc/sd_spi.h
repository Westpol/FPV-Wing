/*
 * sd_spi.h
 *
 *  Created on: Apr 29, 2025
 *      Author: benno
 */

#ifndef INC_SD_SPI_H_
#define INC_SD_SPI_H_

#include "stm32f7xx_hal.h"
#include "ff_gen_drv.h"
#include "ff.h"
#include "diskio.h"
#include <stdint.h>

DSTATUS SD_SPI_Initialize(BYTE pdrv);
DSTATUS SD_SPI_Status(BYTE pdrv);
DRESULT SD_SPI_ReadBlocks(uint8_t *buffer, uint32_t sector, uint32_t count);
DRESULT SD_SPI_WriteBlocks(const uint8_t *buffer, uint32_t sector, uint32_t count);
DRESULT SD_SPI_Ioctl(BYTE cmd, void *buff);
void SD_SPI_Process(void);  // For non-blocking FSM


#endif /* INC_SD_SPI_H_ */
