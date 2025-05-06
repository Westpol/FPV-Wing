/*
 * sd_diskio.c
 *
 *  Created on: Apr 29, 2025
 *      Author: benno
 */

#include "sd_spi.h"
#include "stdint.h"

static BYTE STA = STA_NOINIT;
static SPI_HandleTypeDef* sd_spi;

static void CS_SELECT(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

static void CS_DESELECT(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

static uint8_t SPI_TxRx_Single(uint8_t data) {
    uint8_t received;
    HAL_SPI_TransmitReceive(sd_spi, &data, &received, 1, SPI_TIMEOUT);
    return received;
}

static void SPI_TxRx_Multiple(const uint8_t *txBuf, uint8_t *rxBuf, size_t len) {
    HAL_SPI_TransmitReceive(sd_spi, (uint8_t *)txBuf, rxBuf, len, SPI_TIMEOUT);
}

static void SPI_Transmit_Multiple(const uint8_t *txBuf, size_t len) {
    HAL_SPI_Transmit(sd_spi, (uint8_t *)txBuf, len, SPI_TIMEOUT);
}

static void SPI_Receive_Multiple(uint8_t *rxBuf, size_t len) {
    HAL_SPI_Receive(sd_spi, rxBuf, len, SPI_TIMEOUT);
}

static uint8_t send_cmd(uint8_t cmd, uint32_t arg) {
    uint8_t crc = (cmd == 0x40) ? 0x95 : (cmd == 0x48) ? 0x87 : 0x01;
    SPI_TxRx_Single(cmd);
    SPI_TxRx_Single(arg >> 24); SPI_TxRx_Single(arg >> 16);
    SPI_TxRx_Single(arg >> 8); SPI_TxRx_Single(arg);
    SPI_TxRx_Single(crc);

    for (int i = 0; i < 10; i++) {
        uint8_t res = SPI_TxRx_Single(0xFF);
        if (!(res & 0x80)) return res;
    }
    return 0xFF;
}

void SD_SPI_INIT(SPI_HandleTypeDef* HSPIx){
	sd_spi = HSPIx;
}

DSTATUS SD_SPI_Initialize(BYTE pdrv){
	if(pdrv != 0) return STA_NOINIT;
	if (!(STA & STA_NOINIT)) return STA;

	CS_DESELECT();
	for (int i = 0; i < 10; i++) SPI_TxRx_Single(0xFF);	// send 80 dummy clocks

	CS_SELECT();
	if (send_cmd(0x40, 0) != 0x01) {
		CS_DESELECT();
		return STA;
	}
	CS_DESELECT();

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
