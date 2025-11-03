/*
 * load_config.c
 *
 *  Created on: Oct 8, 2025
 *      Author: benno
 */

#include "load_config.h"

#include <stdint.h>

#include "sd.h"

#include "attitude_pid.h"			// includes to so that I can extern structs
#include "crossfire.h"
#include "flight_control.h"
#include "onboard-sensors.h"
#include "m10-gps.h"
#include "sd_logger.h"
#include "debug.h"
#include "main.h"
#include "config_data.h"

SD_LOGGER_CONFIG_DATA sd_logger_config_data;
CONFIG_HEADER config_header;
CRSF_CHANNELS_CONFIG_DATA crsf_channels_config_header;

uint8_t CONFIG_READ = 0;


static uint32_t next_magic(uint32_t prev) {
    uint32_t x = prev;
    x ^= (x << 13);
    x ^= (x >> 17);
    x ^= (x << 5);
    return x | 0x80000000; // ensure non-zero and high bit set
}

void print_block_blocking(uint32_t block){
	uint8_t bytes[512];

	SD_READ_BLOCK(bytes, block);
	for(int i = 0; i < 32; i++){
		USB_PRINTLN_BLOCKING("%02X %02X %02X %02X %02X %02X %02X %02X    %02X %02X %02X %02X %02X %02X %02X %02X", bytes[0+i*16], bytes[1+i*16], bytes[2+i*16], bytes[3+i*16], bytes[4+i*16], bytes[5+i*16], bytes[6+i*16], bytes[7+i*16], bytes[8+i*16], bytes[9+i*16], bytes[10+i*16], bytes[11+i*16], bytes[12+i*16], bytes[13+i*16], bytes[14+i*16], bytes[15+i*16]);
	}
}

void LOAD_CONFIG_INIT(){
#ifdef CONFIG_WRITE_STANDARD_ENABLED
	CONFIG_WRITE_STANDARD_CONFIG();
#endif
	CONFIG_READ_CONFIG();
}

uint8_t CONFIG_WAS_READ(){
	return CONFIG_READ;
}

static void CONFIG_WRITE_READ_DATA(){
	CONFIG_DATA.logger.log_mode = sd_logger_config_data.log_mode;

	CONFIG_DATA.crossfire.channels.throttle = crsf_channels_config_header.throttle;
	CONFIG_DATA.crossfire.channels.roll = crsf_channels_config_header.roll;
	CONFIG_DATA.crossfire.channels.pitch = crsf_channels_config_header.pitch;
	CONFIG_DATA.crossfire.channels.arm_switch = crsf_channels_config_header.arm_switch;
	CONFIG_DATA.crossfire.channels.mode_switch = crsf_channels_config_header.mode_switch;
}

void CONFIG_READ_CONFIG(){
	uint8_t block_data[512];
	uint32_t magic = START_MAGIC;
	uint32_t block = CONFIG_START_BLOCK;
	SD_READ_BLOCK(block_data, CONFIG_START_BLOCK);

	memcpy(&config_header, &block_data[0], sizeof(config_header));
	if(!(config_header.magic_start == magic && config_header.magic_end == ~magic)) ERROR_HANDLER_BLINKS(2);
	magic = next_magic(magic);
	if(config_header.block_num_next_datastruct != block){
		SD_READ_BLOCK(block_data, config_header.block_num_next_datastruct);
		block = config_header.block_num_next_datastruct;
	}

	memcpy(&sd_logger_config_data, &block_data[config_header.index_next_datastruct], sizeof(sd_logger_config_data));
	if(!(sd_logger_config_data.magic_start == magic && sd_logger_config_data.magic_end == ~magic)) ERROR_HANDLER_BLINKS(3);
	magic = next_magic(magic);
	if(sd_logger_config_data.block_num_next_datastruct != block){
		SD_READ_BLOCK(block_data, sd_logger_config_data.block_num_next_datastruct);
		block = sd_logger_config_data.block_num_next_datastruct;
	}

	memcpy(&crsf_channels_config_header, &block_data[sd_logger_config_data.index_next_datastruct], sizeof(crsf_channels_config_header));
	if(!(crsf_channels_config_header.magic_start == magic && crsf_channels_config_header.magic_end == ~magic)) ERROR_HANDLER_BLINKS(4);
	magic = next_magic(magic);
	if(crsf_channels_config_header.block_num_next_datastruct != block){
		SD_READ_BLOCK(block_data, crsf_channels_config_header.block_num_next_datastruct);
		block = crsf_channels_config_header.block_num_next_datastruct;
	}

	CONFIG_WRITE_READ_DATA();
	CONFIG_READ = 1;
}

#ifdef CONFIG_WRITE_STANDARD_ENABLED
static void INCREASE_INDEX_NEXT_STRUCT(uint16_t current_struct_size, uint16_t next_struct_size, uint16_t* block_index_pos, uint32_t* block){
	if(*block_index_pos + current_struct_size >= 508){
		ERROR_HANDLER_BLINKS(1);
		return;
	}
	if(*block_index_pos + current_struct_size + next_struct_size <= 508){
		*block_index_pos += current_struct_size;
		return;
	}
	*block += 1;
	*block_index_pos = 0;
	return;
}

static void CONFIG_SET_STANDARD_VALUES(){
	uint16_t block_index_pos = 0;
	uint32_t block = CONFIG_START_BLOCK;
	uint32_t magic = START_MAGIC;
	config_header.magic_start = magic;
	config_header.magic_end = ~magic;
	magic = next_magic(magic);
	config_header.num_config_blocks = 1;
	config_header.version = 0;

	INCREASE_INDEX_NEXT_STRUCT(sizeof(config_header), sizeof(sd_logger_config_data), &block_index_pos, &block);
	config_header.index_next_datastruct = block_index_pos;
	config_header.block_num_next_datastruct = block;


	sd_logger_config_data.magic_start = magic;		// sd logger setup
	sd_logger_config_data.magic_end = ~magic;
	magic = next_magic(magic);
	sd_logger_config_data.log_mode = 1;

	INCREASE_INDEX_NEXT_STRUCT(sizeof(sd_logger_config_data), sizeof(crsf_channels_config_header), &block_index_pos, &block);
	sd_logger_config_data.index_next_datastruct = block_index_pos;
	sd_logger_config_data.block_num_next_datastruct = block;


	crsf_channels_config_header.magic_start = magic;
	crsf_channels_config_header.magic_end = ~magic;
	magic = next_magic(magic);
	crsf_channels_config_header.throttle = 0;
	crsf_channels_config_header.roll = 1;
	crsf_channels_config_header.pitch = 2;
	crsf_channels_config_header.arm_switch = 11;
	crsf_channels_config_header.mode_switch = 5;

	INCREASE_INDEX_NEXT_STRUCT(sizeof(crsf_channels_config_header), 0, &block_index_pos, &block);
	crsf_channels_config_header.index_next_datastruct = block_index_pos;
	crsf_channels_config_header.block_num_next_datastruct = block;

}

void CONFIG_WRITE_STANDARD_CONFIG(){
	CONFIG_SET_STANDARD_VALUES();

	uint32_t block = CONFIG_START_BLOCK;
	uint16_t index = 0;
	uint8_t block_byte_array[508] = {0};

	memcpy(&block_byte_array[index], &config_header, sizeof(config_header));
	index = config_header.index_next_datastruct;
	if(config_header.block_num_next_datastruct != block){
		SD_WRITE_BLOCK(block_byte_array, sizeof(block_byte_array), block);
		memset(block_byte_array, 0, sizeof(block_byte_array));
		block = config_header.block_num_next_datastruct;
	}

	memcpy(&block_byte_array[index], &sd_logger_config_data, sizeof(sd_logger_config_data));
	index = sd_logger_config_data.index_next_datastruct;
	if(sd_logger_config_data.block_num_next_datastruct != block){
		SD_WRITE_BLOCK(block_byte_array, 508, block);
		memset(block_byte_array, 0, sizeof(block_byte_array));
		block = sd_logger_config_data.block_num_next_datastruct;
	}

	memcpy(&block_byte_array[index], &crsf_channels_config_header, sizeof(crsf_channels_config_header));
		index = crsf_channels_config_header.index_next_datastruct;
		if(crsf_channels_config_header.block_num_next_datastruct != block){
			SD_WRITE_BLOCK(block_byte_array, 508, block);
			memset(block_byte_array, 0, sizeof(block_byte_array));
			block = crsf_channels_config_header.block_num_next_datastruct;
		}

	SD_WRITE_BLOCK(block_byte_array, sizeof(block_byte_array), block);

}
#endif
