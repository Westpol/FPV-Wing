/*
 * sd_logger.c
 *
 *  Created on: May 6, 2025
 *      Author: benno
 *
 *      TODO: Superblock rotation (reserve 30 blocks for superblock, first block is index of the current active super block, each active superblock has counter, move forward if counter hits value)
 */

#include "sd_logger.h"
#include "debug.h"
#include "stm32f7xx_hal.h"
#include "main.h"

static Sensor_Data* sensor_data;
static CRSF_DATA* crsf_data;
static GPS_NAV_PVT* gps_nav_pvt;

static bool armed = false;
static bool last_arm_status = false;

static SD_SUPERBLOCK sd_superblock = {0};
extern SD_HandleTypeDef hsd1;
extern CRC_HandleTypeDef hcrc;



static uint32_t calculate_crc32_hw(const void *data, size_t length) {
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

void SD_LOGGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT){


	// Needs to be 32-byte aligned due to D-Cache
	uint8_t superblock_raw[BLOCK_SIZE];

	    /*// 2. Clean D-Cache before DMA access
	    SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer, ((BLOCK_SIZE + 31) / 32) * 32);*/

	    // 3. Optional: Check card state
	    if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
	        ERROR_HANDLER_BLINKS(1); // Not ready
	    }

	    // check init
	    if (HAL_SD_Init(&hsd1) != HAL_OK) {
	        ERROR_HANDLER_BLINKS(4);  // Init failed
	    }

	    // 4. Write block
	    if (HAL_SD_ReadBlocks(&hsd1, superblock_raw, SUPERBLOCK_BLOCK, 1, TIMEOUT_MS) != HAL_OK) {
	        ERROR_HANDLER_BLINKS(2); // Write failed
	    }



	    memcpy(&sd_superblock, &superblock_raw, sizeof(sd_superblock));

	    if(sd_superblock.magic != SUPERBLOCK_MAGIC){
	    	ERROR_HANDLER_BLINKS(10);
	    }

	    if(sd_superblock.crc32 != calculate_crc32_hw(&sd_superblock, sizeof(sd_superblock) - sizeof(uint32_t))){
	    	ERROR_HANDLER_BLINKS(10);
	    }

	    // 5. Wait until the card is ready again
	    uint32_t start = HAL_GetTick();
	    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
	        if (HAL_GetTick() - start > TIMEOUT_MS) {
	            ERROR_HANDLER_BLINKS(3); // Timeout
	        }
	    }

	sensor_data = SENSOR_DATA;
	crsf_data = CRSF_DATA;
	gps_nav_pvt = GPS_NAV_PVT;

}

void SD_LOGGER_LOOP_CALL(){
	last_arm_status = false;

	/*
	// open file at arm
	if(last_arm_status == false && armed == true){
		last_arm_status = true;
		res = f_open(&file, "/testfile.bin", FA_OPEN_ALWAYS | FA_WRITE);
		if(res != FR_OK){
			Error_Handler();
		}
		else if(res == FR_OK){
			res = f_lseek(&file, f_size(&file));
			if(res != FR_OK){
				Error_Handler();
			}
		}
		STATUS_LED_GREEN_ON();
		return;
	}
	// log file while armed
	if(armed && last_arm_status){
		unsigned int bytes_written;
		//res = f_write(&file, &crsf_data->channel[0], sizeof(crsf_data->channel[0]), &bytes_written);
		uint8_t test_buf[] = {1, 2, 3, 4, 5};
		res = f_write(&file, test_buf, sizeof(test_buf), &bytes_written);
		if(res != FR_OK){
			Error_Handler();
		}
		return;
	}
	// close file after disarm
	if(last_arm_status == true && armed == false){
		last_arm_status = false;
		res = f_sync(&file);
		if(res != FR_OK){
			Error_Handler();
		}
		res = f_close(&file);
		if(res != FR_OK){
			Error_Handler();
		}
		STATUS_LED_GREEN_OFF();
		return;
	}*/
}

void SD_LOGGER_FORWARD_ARM(bool ARM_STATUS){
	armed = ARM_STATUS;
}

void SD_LOGGER_SETUP_CARD(){
	SD_SUPERBLOCK sd_superblock_config = {0};
	sd_superblock_config.magic = SUPERBLOCK_MAGIC;
	sd_superblock_config.version = SUPERBLOCK_VERSION;
	sd_superblock_config.file_start_block = 1000;
	sd_superblock_config.file_end_block = 60000000;
	sd_superblock_config.card_size_MB = 32000;
	sd_superblock_config.total_flights = 0;
	sd_superblock_config.last_flight_number = 0;
	sd_superblock_config.corruption_flag = 0;
	sd_superblock_config.latest_metadata_block = 101;
	sd_superblock_config.crc32 = calculate_crc32_hw(&sd_superblock, sizeof(sd_superblock) - sizeof(uint32_t));

	if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
		ERROR_HANDLER_BLINKS(10); // Not ready
	}

	if (HAL_SD_WriteBlocks(&hsd1, (uint8_t *)&sd_superblock_config, SUPERBLOCK_BLOCK, 1, TIMEOUT_MS) != HAL_OK) {
		ERROR_HANDLER_BLINKS(10); // Write failed
	}

}
