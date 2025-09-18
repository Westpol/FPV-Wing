/*
 * sd_logger.c
 *
 *  Created on: May 6, 2025
 *      Author: benno
 *
 * ## ✅ Done
 * - Basic log config struct
 * - Read mission block from SD
 *
 * ## 🔨 In Progress
 * - Implement log behavior change based on config
 *
 * ## 🧠 Future
 * - Per-sensor field mask
 * - Waypoint block indexing
 * - PC-side mission editor
 *
 * sudo dd if=/dev/sde bs=512 skip=99 count=1 status=none | hexdump -C
 *
 *
 *      TODO: Superblock rotation (reserve 30 blocks for superblock, first block is index of the current active super block, each active superblock has counter, move forward if counter hits value)
 */

#include "sd_logger.h"
#include "debug.h"
#include "stm32f7xx_hal.h"
#include "main.h"
#include "logging_packager.h"
#include "flight_control.h"
#include "m10-gps.h"
#include "flight_state.h"

static bool last_arm_status = false;

static uint8_t log_buffer_1[2048] __attribute__((aligned(32))) = {0};
static uint8_t log_buffer_2[2048] __attribute__((aligned(32))) = {0};
static uint8_t current_buffer = 0;
static uint16_t buffer_index = 0;
static uint8_t buffer_block = 0;
static uint8_t* active_log_buffer = &log_buffer_1[0];

static uint32_t latest_metadata_block = 0;
static uint8_t latest_metadata_index = 0;
static uint8_t current_metadata_index = 0;
static uint8_t log_mode = LOG_TYPE_DISABLE_LOGGING;

static uint32_t last_log_block;
static SD_SUPERBLOCK sd_superblock = {0};
static SD_FILE_METADATA_BLOCK sd_file_metadata_block = {0};
static uint8_t raw_block_data[BLOCK_SIZE];
extern SD_HandleTypeDef hsd1;
extern CRC_HandleTypeDef hcrc;
extern GPS_DATA gps_data;

static volatile bool dma_busy = false;


// DMA complete callback
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    dma_busy = false;
}

static uint32_t CALCULATE_CRC32_HW(const void *data, size_t length) {
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
	uint32_t calculated_crc = CALCULATE_CRC32_HW(data, size);
	if(calculated_crc != expected_crc){
		DEBUG_PRINT_VERBOSE("Calculated CRC: %08X\nExpected CRC: %08X", calculated_crc, expected_crc);
		LOG_FAIL_WITH_ERROR(ERROR_CRC_MISMATCH);
	}
}

static void READ_BLOCK(uint8_t* data_storage, uint32_t block){

	uint8_t read_buffer[BLOCK_SIZE] = {0};

	uint32_t start = HAL_GetTick();
	while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
		if (HAL_GetTick() - start > TIMEOUT_MS) {
			LOG_FAIL_WITH_ERROR(ERROR_TIMEOUT); // Timeout
		}
	}
	if(HAL_SD_ReadBlocks(&hsd1, read_buffer, block, 1, TIMEOUT_MS) != HAL_OK){
		LOG_FAIL_WITH_ERROR(ERROR_WRITE); // Write failed
	}

	uint32_t block_crc32 = ((uint32_t)read_buffer[511] << 24) | ((uint32_t)read_buffer[510] << 16) | ((uint32_t)read_buffer[509] << 8)  | ((uint32_t)read_buffer[508]);
	VERIFY_CRC32(read_buffer, BLOCK_SIZE - CRC32_BYTE_SIZE, block_crc32);
	memcpy(data_storage, read_buffer, BLOCK_SIZE);

}

static void WRITE_BLOCK(uint8_t* data_array, uint32_t data_length_bytes, uint32_t block){

	if (data_length_bytes > BLOCK_SIZE - CRC32_BYTE_SIZE) {
		LOG_FAIL_WITH_ERROR(ERROR_BLOCK_LIMIT_REACHED); // Too much data
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
			LOG_FAIL_WITH_ERROR(ERROR_TIMEOUT); // Timeout
		}
	}
	if(HAL_SD_WriteBlocks(&hsd1, single_write_buffer, block, 1, TIMEOUT_MS) != HAL_OK){
		LOG_FAIL_WITH_ERROR(ERROR_WRITE); // Write failed
	}
}

static uint8_t WRITE_BUFFER_DMA(uint32_t start_block){
    if (dma_busy) {
        // DMA still writing, defer
        return 1; // Deferred
    }

    if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        // Card not ready, defer
        return 2; // Deferred
    }

    // Clear D-Cache for DMA consistency
    SCB_CleanDCache_by_Addr((uint32_t*)((current_buffer == 0) ? log_buffer_1 : log_buffer_2), 2048);

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(&hsd1, (current_buffer == 0) ? log_buffer_1 : log_buffer_2, start_block, 4);

    if (status != HAL_OK) {
        return 3; // Failed
    }

    dma_busy = true;
    return 0; // Success
}

static uint32_t FLIGHT_NUM_TO_BLOCK(uint32_t relative_flight_num){
	uint32_t block = 0;

	block = LOG_METADATA_BLOCK_START + relative_flight_num / LOG_FILES_PER_METADATA_BLOCK;

	return block;
}

static uint8_t FLIGHT_NUM_TO_INDEX(uint32_t relative_flight_num){
	uint8_t index = 0;

	index = relative_flight_num % LOG_FILES_PER_METADATA_BLOCK;

	return index;
}

static void READ_LATEST_FLIGHT(){
	READ_BLOCK(raw_block_data, SUPERBLOCK_BLOCK);
	memcpy(&sd_superblock, &raw_block_data, sizeof(sd_superblock));
	if(sd_superblock.magic != SUPERBLOCK_MAGIC){
		LOG_FAIL_WITH_ERROR(ERROR_WRONG_MAGIC);
	}
	DEBUG_PRINT_VERBOSE("Superblock magic number: 0x%08X correct!", sd_superblock.magic);
	DEBUG_PRINT_VERBOSE("Superblock version: %d\r\nLast Flight Num: %d\r\n", sd_superblock.version, sd_superblock.absolute_flight_num);

	latest_metadata_block = FLIGHT_NUM_TO_BLOCK(sd_superblock.relative_flight_num - 1);
	latest_metadata_index = FLIGHT_NUM_TO_INDEX(sd_superblock.relative_flight_num - 1);
	log_mode = sd_superblock.log_mode_flag;

	if(sd_superblock.relative_flight_num == 0){		// hard fix bug on first flight because system is built on an existing flight before
		READ_BLOCK(raw_block_data, LOG_METADATA_BLOCK_START);
		memcpy(&sd_file_metadata_block, &raw_block_data, sizeof(sd_file_metadata_block));
		if(sd_file_metadata_block.magic != LOG_METADATA_BLOCK_MAGIC){
			LOG_FAIL_WITH_ERROR(ERROR_WRONG_MAGIC);
		}

		DEBUG_PRINT_VERBOSE("Metadata magic number: 0x%08X correct!", sd_file_metadata_block.magic);
		current_metadata_index = 0;
		latest_metadata_block = LOG_METADATA_BLOCK_START;
		return;
	}

	READ_BLOCK(raw_block_data, latest_metadata_block);
	memcpy(&sd_file_metadata_block, &raw_block_data, sizeof(sd_file_metadata_block));
	if(sd_file_metadata_block.magic != LOG_METADATA_BLOCK_MAGIC){
		LOG_FAIL_WITH_ERROR(ERROR_WRONG_MAGIC);
	}
	DEBUG_PRINT_VERBOSE("Metadata magic number: 0x%08X correct!", sd_file_metadata_block.magic);


	if(latest_metadata_index < 13){
		sd_file_metadata_block.sd_file_metadata_chunk[latest_metadata_index + 1].start_block = sd_file_metadata_block.sd_file_metadata_chunk[latest_metadata_index].end_block + 1;
		current_metadata_index = latest_metadata_index + 1;
	}
	else{
		latest_metadata_index = 0;
		current_metadata_index = 0;
		latest_metadata_block += 1;
		uint32_t metadata_block_switch_old_end_block = sd_file_metadata_block.sd_file_metadata_chunk[latest_metadata_index].end_block;
		SD_FILE_METADATA_CHUNK temporary_dummy = {0};
		temporary_dummy.magic = LOG_METADATA_MAGIC;
		temporary_dummy.version = 1;
		temporary_dummy.active_flag = 0;
		temporary_dummy.log_finished = 0;

		for(int i = 0; i < LOG_FILES_PER_METADATA_BLOCK; i++){
			sd_file_metadata_block.sd_file_metadata_chunk[i] = temporary_dummy;
		}
		sd_file_metadata_block.magic = LOG_METADATA_BLOCK_MAGIC;

		sd_file_metadata_block.sd_file_metadata_chunk[latest_metadata_index].start_block = metadata_block_switch_old_end_block + 1;

	}

}

uint32_t SD_LOGGER_INIT(){


	// Needs to be 32-byte aligned due to D-Cache


	/*// 2. Clean D-Cache before DMA access
	SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer, ((BLOCK_SIZE + 31) / 32) * 32);*/
	// check init


	//SD_LOGGER_SETUP_CARD();
	READ_LATEST_FLIGHT();

	return LOGGING_PACKER_INTERVAL_MICROSECONDS(log_mode);


}

void SD_LOGGER_LOOP_CALL(){
	if(log_mode == LOG_TYPE_DISABLE_LOGGING) return;
	// open file at arm
	if(last_arm_status == false && FLIGHT_STATE_IS_ARMED() == true){
		last_arm_status = true;
		READ_LATEST_FLIGHT();
		if(log_mode == LOG_TYPE_DISABLE_LOGGING) return;	// in case something went wrong in READ_LATEST_FLIGHT

		sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].active_flag = true;
		sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].flight_number = sd_superblock.absolute_flight_num;
		sd_superblock.relative_flight_num += 1;
		sd_superblock.absolute_flight_num += 1;
		last_log_block = sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].start_block;

		sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].log_version = LOG_VERSION;
		sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].log_mode = log_mode;
		sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].timestamp_unix = gps_data.unix_timestamp;

		DEBUG_PRINT_VERBOSE("Current: Metadata block: %d\r\nFlight num: %d\r\nStart block: %d\r\nEnd block: %d", latest_metadata_block, sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].flight_number, sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].start_block, sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].end_block);

		WRITE_BLOCK((uint8_t*)&sd_file_metadata_block, sizeof(sd_file_metadata_block), latest_metadata_block);
		WRITE_BLOCK((uint8_t*)&sd_superblock, sizeof(sd_superblock), SUPERBLOCK_BLOCK);

		STATUS_LED_GREEN_ON();
		memset(log_buffer_1, 0, sizeof(log_buffer_1));
		memset(log_buffer_2, 0, sizeof(log_buffer_2));
		buffer_index = 0;
		current_buffer = 0;
		buffer_block = 0;
		active_log_buffer = log_buffer_1;
		return;
	}
	// log file while arm_status
	if(FLIGHT_STATE_IS_ARMED() == true && last_arm_status == true){
		uint8_t* array_pointer = LOGGING_PACKER_BY_MODE(log_mode);

		uint8_t array_length = array_pointer[0];

		if(buffer_index + array_length <= 512 - sizeof(CRC32_BYTE_SIZE)){
			memcpy(active_log_buffer + buffer_index + (512 * buffer_block), array_pointer + 1, array_length);
			buffer_index += array_length;
		}
		else{
			uint32_t crc = CALCULATE_CRC32_HW(active_log_buffer + (512 * buffer_block), 508);
			memcpy(active_log_buffer + 508 + (512 * buffer_block), &crc, sizeof(crc));
			buffer_index = 0;
			buffer_block += 1;
			if(buffer_block > 3){
				// change buffers, write buffer to SD card, write value other buffer
				if(WRITE_BUFFER_DMA(last_log_block) != 0){
					LOG_FAIL_WITH_ERROR(ERROR_DMA_WRITE);		// conditional executions
				}
				last_log_block += 4;
				DEBUG_PRINT_VERBOSE("Block %d", last_log_block);
				buffer_block = 0;
				buffer_index = 0;
				if(current_buffer == 0){
					active_log_buffer = &log_buffer_2[0];
					current_buffer = 1;
				}
				else{
					active_log_buffer = &log_buffer_1[0];
					current_buffer = 0;
				}

			}
			else{		// continue filling the next buffer
				memcpy(active_log_buffer + buffer_index + (512 * buffer_block), array_pointer + 1, array_length);
				buffer_index += array_length;
			}
		}

	}
	// close file after disarm
	if(last_arm_status == true && FLIGHT_STATE_IS_ARMED() == false){

		// TODO: need to add flush rest of buffer

		sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].end_block = last_log_block;
		sd_file_metadata_block.sd_file_metadata_chunk[current_metadata_index].log_finished = 1;

		WRITE_BLOCK((uint8_t*)&sd_superblock, sizeof(sd_superblock), SUPERBLOCK_BLOCK);
		WRITE_BLOCK((uint8_t*)&sd_file_metadata_block, sizeof(sd_file_metadata_block), latest_metadata_block);
		last_arm_status = false;
		STATUS_LED_GREEN_OFF();
		return;
	}
}


void SD_LOGGER_SETUP_CARD(){
	SD_SUPERBLOCK sd_superblock_config = {0};
	SD_FILE_METADATA_BLOCK sd_file_metadata_block_config = {0};
	sd_superblock_config.magic = SUPERBLOCK_MAGIC;
	sd_superblock_config.version = SUPERBLOCK_VERSION;

	sd_superblock_config.log_metadata_start_block = LOG_METADATA_BLOCK_START;
	sd_superblock_config.log_metadata_end_block = LOG_METADATA_BLOCK_END;
	sd_superblock_config.logfile_start_block = LOG_DATA_BLOCK_START;
	sd_superblock_config.logfile_end_block = LOG_DATA_BLOCK_END;

	sd_superblock_config.mission_metadata_start_block = MISSION_METADATA_BLOCK_START;
	sd_superblock_config.mission_metadata_end_block = MISSION_METADATA_BLOCK_END;
	sd_superblock_config.missionfile_start_block = MISSION_DATA_BLOCK_START;
	sd_superblock_config.missionfile_end_block = MISSION_DATA_BLOCK_END;

	sd_superblock_config.card_size_MB = 32000;
	sd_superblock_config.absolute_flight_num = 0;
	sd_superblock_config.relative_flight_num = 0;
	sd_superblock_config.corruption_flag = 0;
	sd_superblock_config.latest_mission_metadata_block = MISSION_METADATA_BLOCK_START;

	sd_superblock_config.active_mission_id = 0;		// standard mission with no waypoints, etc.
	sd_superblock_config.log_mode_flag = LOG_TYPE_T1V0_GENERAL;

	SD_FILE_METADATA_CHUNK temporary_dummy = {0};
	temporary_dummy.magic = LOG_METADATA_MAGIC;
	temporary_dummy.version = 1;
	temporary_dummy.active_flag = 0;
	temporary_dummy.log_finished = 0;

	for(int i = 0; i < LOG_FILES_PER_METADATA_BLOCK; i++){
		sd_file_metadata_block_config.sd_file_metadata_chunk[i] = temporary_dummy;
	}

	sd_file_metadata_block_config.sd_file_metadata_chunk[0].start_block = LOG_DATA_BLOCK_START;

	sd_file_metadata_block_config.magic = LOG_METADATA_BLOCK_MAGIC;

	WRITE_BLOCK((uint8_t *)&sd_superblock_config, sizeof(sd_superblock_config), SUPERBLOCK_BLOCK);

	WRITE_BLOCK((uint8_t *)&sd_file_metadata_block_config, sizeof(sd_file_metadata_block_config), LOG_METADATA_BLOCK_START);
}
