/*
 * sd.c
 *
 *  Created on: Sep 12, 2025
 *      Author: benno
 */

#include "sd.h"
#include "stm32f7xx_hal.h"
#include <string.h>
#include "main.h"

extern SD_HandleTypeDef hsd1;
extern CRC_HandleTypeDef hcrc;

static volatile bool dma_busy = false;

static void LOG_FAIL_WITH_ERROR(uint8_t error_code){
#if DEBUG_ENDABLED
	ERROR_HANDLER_BLINKS(error_code);
	return;
#else
	return;
#endif
}

// DMA complete callback
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    dma_busy = false;
}


uint32_t CALCULATE_CRC32_HW(const void *data, size_t length) {
    // STM32 CRC peripheral processes 32-bit words, so we need to handle padding
    size_t aligned_length = length & ~0x3;  // Number of full 32-bit words
    size_t remaining_bytes = length & 0x3;

    // Reset CRC calculator
    HAL_CRC_Init(&hcrc);
    __HAL_CRC_DR_RESET(&hcrc);

    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, aligned_length / 4);

    // Handle remaining bytes manually if not aligned
    if (remaining_bytes > 0) {
        uint8_t tail[4] = {0};
        memcpy(tail, (uint8_t*)data + aligned_length, remaining_bytes);
        crc = HAL_CRC_Accumulate(&hcrc, (uint32_t*)tail, 1);
    }

    return crc;
}

void VERIFY_CRC32(const void* data, size_t size, uint32_t expected_crc){
	uint32_t calculated_crc = CALCULATE_CRC32_HW(data, size);
	if(calculated_crc != expected_crc){
		DEBUG_PRINT_VERBOSE("Calculated CRC: %08X\nExpected CRC: %08X", calculated_crc, expected_crc);
		LOG_FAIL_WITH_ERROR(SD_ERROR_CRC_MISMATCH);
	}
}

void SD_READ_BLOCK(uint8_t* data_storage, uint32_t block){

	uint8_t read_buffer[BLOCK_SIZE] = {0};

	uint32_t start = HAL_GetTick();
	while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
		if (HAL_GetTick() - start > TIMEOUT_MS) {
			LOG_FAIL_WITH_ERROR(SD_ERROR_TIMEOUT); // Timeout
		}
	}
	if(HAL_SD_ReadBlocks(&hsd1, read_buffer, block, 1, TIMEOUT_MS) != HAL_OK){
		LOG_FAIL_WITH_ERROR(SD_ERROR_WRITE); // Write failed
	}

	uint32_t block_crc32 = ((uint32_t)read_buffer[511] << 24) | ((uint32_t)read_buffer[510] << 16) | ((uint32_t)read_buffer[509] << 8)  | ((uint32_t)read_buffer[508]);
	VERIFY_CRC32(read_buffer, BLOCK_SIZE - CRC32_BYTE_SIZE, block_crc32);
	memcpy(data_storage, read_buffer, BLOCK_SIZE);

}

void SD_WRITE_BLOCK(uint8_t* data_array, uint32_t data_length_bytes, uint32_t block){

	if (data_length_bytes > BLOCK_SIZE - CRC32_BYTE_SIZE) {
		LOG_FAIL_WITH_ERROR(SD_ERROR_BLOCK_LIMIT_REACHED); // Too much data
	}

	uint8_t single_write_buffer[BLOCK_SIZE] = {0};
	memcpy(single_write_buffer, data_array, data_length_bytes);
	uint32_t crc32 = CALCULATE_CRC32_HW(single_write_buffer, BLOCK_SIZE - CRC32_BYTE_SIZE);
	single_write_buffer[508] = (crc32 >> 0);
	single_write_buffer[509] = (crc32 >> 8);
	single_write_buffer[510] = (crc32 >> 16);
	single_write_buffer[511] = (crc32 >> 24);

	uint32_t start = HAL_GetTick();
	while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
		if (HAL_GetTick() - start > TIMEOUT_MS) {
			LOG_FAIL_WITH_ERROR(SD_ERROR_TIMEOUT); // Timeout
		}
	}
	if(HAL_SD_WriteBlocks(&hsd1, single_write_buffer, block, 1, TIMEOUT_MS) != HAL_OK){
		LOG_FAIL_WITH_ERROR(SD_ERROR_WRITE); // Write failed
	}
}

SD_DMA_STATUS SD_WRITE_DMA(uint8_t* data_array, uint32_t block, uint32_t num_blocks){
	if(SD_DMA_BUSY()){
		return SD_DMA_BUSY_NOT_STARTED;
	}

    if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        // Card not ready, defer
        return SD_CARD_BUSY_NOT_STARTED; // Deferred
    }

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(&hsd1, data_array, block, num_blocks);

    if(status != HAL_OK){
    	return SD_DMA_ERROR_NOT_STARTED;
    }

    dma_busy = true;

	return SD_DMA_TRANSMISSION_STARTED;
}

bool SD_DMA_BUSY(void){
	return dma_busy;
}
