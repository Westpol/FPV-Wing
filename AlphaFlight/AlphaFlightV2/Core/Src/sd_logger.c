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
static SD_FILE_METADATA_CHUNK new_file_metadata = {0};
static SD_FILE_METADATA_BLOCK sd_file_metadata_block = {0};
static uint8_t raw_block_data[BLOCK_SIZE];
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

static inline void VERIFY_CRC32(const void* data, size_t size, uint32_t expected_crc){
	if(calculate_crc32_hw(data, size) != expected_crc){
		ERROR_HANDLER_BLINKS(10);
	}
}

static void READ_BLOCK(uint8_t* data_storage, uint32_t start_block, uint32_t num_blocks){
	uint32_t start = HAL_GetTick();
	while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
		if (HAL_GetTick() - start > TIMEOUT_MS) {
			ERROR_HANDLER_BLINKS(3); // Timeout
		}
	}
	if(HAL_SD_ReadBlocks(&hsd1, data_storage, start_block, num_blocks, TIMEOUT_MS) != HAL_OK){
		ERROR_HANDLER_BLINKS(2); // Write failed
	}
}

static void WRITE_BLOCK(uint8_t* data_array, uint32_t start_block, uint32_t num_blocks){
	uint32_t start = HAL_GetTick();
	while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
		if (HAL_GetTick() - start > TIMEOUT_MS) {
			ERROR_HANDLER_BLINKS(3); // Timeout
		}
	}
	if(HAL_SD_WriteBlocks(&hsd1, data_array, start_block, num_blocks, TIMEOUT_MS) != HAL_OK){
		ERROR_HANDLER_BLINKS(2); // Write failed
	}
}

static void READ_LATEST_FLIGHT(){
	READ_BLOCK(raw_block_data, SUPERBLOCK_BLOCK, 1);
	memcpy(&sd_superblock, &raw_block_data, sizeof(sd_superblock));
	if(sd_superblock.magic != SUPERBLOCK_MAGIC) ERROR_HANDLER_BLINKS(10);
	USB_PRINTLN_BLOCKING("Superblock magic number: 0x%08X correct!", sd_superblock.magic);
	VERIFY_CRC32(&sd_superblock, sizeof(sd_superblock) - sizeof(uint32_t), sd_superblock.crc32);
	USB_PRINTLN_BLOCKING("Superblock CRC32: 0x%08X correct!", sd_superblock.crc32);
	USB_PRINTLN_BLOCKING("Superblock version: %d\nCard Size: %d\nLast Flight Num: %d\nLatest Metadata Block: %d", sd_superblock.version, sd_superblock.card_size_MB, sd_superblock.last_flight_number, sd_superblock.latest_metadata_block);

	READ_BLOCK(raw_block_data, sd_superblock.latest_metadata_block, 1);
	memcpy(&sd_file_metadata_block, &raw_block_data, sizeof(sd_file_metadata_block));
	if(sd_file_metadata_block.magic != METADATA_BLOCK_MAGIC) ERROR_HANDLER_BLINKS(10);
	USB_PRINTLN_BLOCKING("Metadata magic number: 0x%08X correct!", sd_file_metadata_block.magic);
	VERIFY_CRC32(&sd_file_metadata_block, sizeof(sd_file_metadata_block) - sizeof(uint32_t), sd_file_metadata_block.crc32);
	USB_PRINTLN_BLOCKING("Metadata CRC32: 0x%08X correct!", sd_file_metadata_block.crc32);

	for(int i = 0; i < FILES_PER_METADATA_BLOCK; i++){
		if(sd_file_metadata_block.sd_file_metadata_chunk[i].active_flag == 0){
			if(sd_file_metadata_block.sd_file_metadata_chunk[i].magic != METADATA_BLOCK_MAGIC){
				ERROR_HANDLER_BLINKS(20);
			}
			new_file_metadata = sd_file_metadata_block.sd_file_metadata_chunk[i];
			new_file_metadata.active_flag = 1;
			new_file_metadata.flight_number = sd_superblock.last_flight_number + 1;
			new_file_metadata.start_block = 0;

		}
		if(sd_file_metadata_block.sd_file_metadata_chunk[i].magic != METADATA_BLOCK_MAGIC){
			ERROR_HANDLER_BLINKS(20);
		}
	}
}

void SD_LOGGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT){


	// Needs to be 32-byte aligned due to D-Cache


	/*// 2. Clean D-Cache before DMA access
	SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer, ((BLOCK_SIZE + 31) / 32) * 32);*/
	// check init


	SD_LOGGER_SETUP_CARD();

	sensor_data = SENSOR_DATA;
	crsf_data = CRSF_DATA;
	gps_nav_pvt = GPS_NAV_PVT;

}

void SD_LOGGER_LOOP_CALL(){
	// open file at arm
	if(last_arm_status == false && armed == true){
		last_arm_status = true;
		READ_LATEST_FLIGHT();
		STATUS_LED_GREEN_ON();
		return;
	}
	// log file while armed
	if(armed && last_arm_status){
		return;
	}
	// close file after disarm
	if(last_arm_status == true && armed == false){
		last_arm_status = false;
		STATUS_LED_GREEN_OFF();
		return;
	}
}

void SD_LOGGER_FORWARD_ARM(bool ARM_STATUS){
	armed = ARM_STATUS;
}

void SD_LOGGER_SETUP_CARD(){
	SD_SUPERBLOCK sd_superblock_config = {0};
	SD_FILE_METADATA_BLOCK sd_file_metadata_block_config = {0};
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

	SD_FILE_METADATA_CHUNK temporary_dummy = {0};
	temporary_dummy.magic = METADATA_MAGIC;
	temporary_dummy.version = 1;
	temporary_dummy.active_flag = 0;
	temporary_dummy.log_finished = 0;

	for(int i = 0; i < FILES_PER_METADATA_BLOCK; i++){
		sd_file_metadata_block_config.sd_file_metadata_chunk[i] = temporary_dummy;
	}
	sd_file_metadata_block_config.magic = METADATA_BLOCK_MAGIC;

	sd_file_metadata_block_config.crc32 = calculate_crc32_hw(&sd_file_metadata_block_config, sizeof(sd_file_metadata_block_config) - sizeof(uint32_t));

	WRITE_BLOCK((uint8_t *)&sd_superblock_config, SUPERBLOCK_BLOCK, 1);

	WRITE_BLOCK((uint8_t *)&sd_file_metadata_block_config, METADATA_BLOCK_START, 1);
}
