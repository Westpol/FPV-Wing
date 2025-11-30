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
CRSF_CHANNELS_CONFIG_DATA crsf_channels_config_data;
PID_VALUES_CONFIG_DATA pid_values_config_data;

uint8_t config_been_read = 0;


static uint32_t next_magic(uint32_t prev) {
    uint32_t x = prev;
    x ^= (x << 13);
    x ^= (x >> 17);
    x ^= (x << 5);
    return x | 0x80000000; // ensure non-zero and high bit set
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
	if(config_been_read == 0){
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
		sd_logger_config_data.log_mode = 3;

		INCREASE_INDEX_NEXT_STRUCT(sizeof(sd_logger_config_data), sizeof(crsf_channels_config_data), &block_index_pos, &block);
		sd_logger_config_data.index_next_datastruct = block_index_pos;
		sd_logger_config_data.block_num_next_datastruct = block;


		crsf_channels_config_data.magic_start = magic;
		crsf_channels_config_data.magic_end = ~magic;
		magic = next_magic(magic);
		crsf_channels_config_data.throttle = 0;
		crsf_channels_config_data.roll = 2;
		crsf_channels_config_data.pitch = 1;
		crsf_channels_config_data.arm_switch = 11;
		crsf_channels_config_data.mode_switch = 5;

		INCREASE_INDEX_NEXT_STRUCT(sizeof(crsf_channels_config_data), sizeof(pid_values_config_data), &block_index_pos, &block);
		crsf_channels_config_data.index_next_datastruct = block_index_pos;
		crsf_channels_config_data.block_num_next_datastruct = block;

		pid_values_config_data.magic_start = magic;
		pid_values_config_data.magic_end = ~magic;
		magic = next_magic(magic);

		pid_values_config_data.pitch_p = 0.1;
		pid_values_config_data.pitch_i = 0.1;
		pid_values_config_data.pitch_d = 0.1;
		pid_values_config_data.pitch_d_filter = 0.1;
		pid_values_config_data.pitch_feed_forward = 0.1;
		pid_values_config_data.pitch_i_limit = 0.3;
		pid_values_config_data.pitch_i_zone = 0.3;
		pid_values_config_data.pitch_multiplier = 0.1;

		pid_values_config_data.roll_p = 0.1;
		pid_values_config_data.roll_i = 0.1;
		pid_values_config_data.roll_d = 0.1;
		pid_values_config_data.roll_d_filter = 0.1;
		pid_values_config_data.roll_feed_forward = 0.1;
		pid_values_config_data.roll_i_limit = 0.3;
		pid_values_config_data.roll_i_zone = 0.3;
		pid_values_config_data.roll_multiplier = 0.1;

		INCREASE_INDEX_NEXT_STRUCT(sizeof(pid_values_config_data), 0, &block_index_pos, &block);
		pid_values_config_data.index_next_datastruct = block_index_pos;
		pid_values_config_data.block_num_next_datastruct = block;
	}

}
#endif


void LOAD_CONFIG_INIT(){
#ifdef CONFIG_WRITE_STANDARD_ENABLED
	CONFIG_SET_STANDARD_VALUES();
	CONFIG_WRITE();
#endif
	CONFIG_READ();
}

uint8_t CONFIG_WAS_READ(){
	return config_been_read;
}

static void CONFIG_SET_CONFIG_DATA_STRUCT(){
	CONFIG_DATA.logger.log_mode = sd_logger_config_data.log_mode;

	CONFIG_DATA.crossfire.channels.throttle = crsf_channels_config_data.throttle;
	CONFIG_DATA.crossfire.channels.roll = crsf_channels_config_data.roll;
	CONFIG_DATA.crossfire.channels.pitch = crsf_channels_config_data.pitch;
	CONFIG_DATA.crossfire.channels.arm_switch = crsf_channels_config_data.arm_switch;
	CONFIG_DATA.crossfire.channels.mode_switch = crsf_channels_config_data.mode_switch;

	CONFIG_DATA.pid.roll.d_filter = pid_values_config_data.roll_d_filter;
	CONFIG_DATA.pid.roll.feed_forward = pid_values_config_data.roll_feed_forward;
	CONFIG_DATA.pid.roll.i = pid_values_config_data.roll_i;
	CONFIG_DATA.pid.roll.i_limit = pid_values_config_data.roll_i_limit;
	CONFIG_DATA.pid.roll.i_zone = pid_values_config_data.roll_i_zone;
	CONFIG_DATA.pid.roll.multiplier = pid_values_config_data.roll_multiplier;
	CONFIG_DATA.pid.roll.p = pid_values_config_data.roll_p;

	CONFIG_DATA.pid.pitch.d_filter = pid_values_config_data.pitch_d_filter;
	CONFIG_DATA.pid.pitch.feed_forward = pid_values_config_data.pitch_feed_forward;
	CONFIG_DATA.pid.pitch.i = pid_values_config_data.pitch_i;
	CONFIG_DATA.pid.pitch.i_limit = pid_values_config_data.pitch_i_limit;
	CONFIG_DATA.pid.pitch.i_zone = pid_values_config_data.pitch_i_zone;
	CONFIG_DATA.pid.pitch.multiplier = pid_values_config_data.pitch_multiplier;
	CONFIG_DATA.pid.pitch.p = pid_values_config_data.pitch_p;
	CONFIG_DATA_BACKUP_DATA();
}

void CONFIG_READ(){
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

	memcpy(&crsf_channels_config_data, &block_data[sd_logger_config_data.index_next_datastruct], sizeof(crsf_channels_config_data));
	if(!(crsf_channels_config_data.magic_start == magic && crsf_channels_config_data.magic_end == ~magic)) ERROR_HANDLER_BLINKS(4);
	magic = next_magic(magic);
	if(crsf_channels_config_data.block_num_next_datastruct != block){
		SD_READ_BLOCK(block_data, crsf_channels_config_data.block_num_next_datastruct);
		block = crsf_channels_config_data.block_num_next_datastruct;
	}

	memcpy(&pid_values_config_data, &block_data[crsf_channels_config_data.index_next_datastruct], sizeof(pid_values_config_data));
	if(!(pid_values_config_data.magic_start == magic && pid_values_config_data.magic_end == ~magic)) ERROR_HANDLER_BLINKS(5);
	magic = next_magic(magic);
	if(pid_values_config_data.block_num_next_datastruct != block){
		SD_READ_BLOCK(block_data, pid_values_config_data.block_num_next_datastruct);
		block = pid_values_config_data.block_num_next_datastruct;
	}

	CONFIG_SET_CONFIG_DATA_STRUCT();
	config_been_read = 1;
}



void CONFIG_WRITE(){

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

	memcpy(&block_byte_array[index], &crsf_channels_config_data, sizeof(crsf_channels_config_data));
		index = crsf_channels_config_data.index_next_datastruct;
		if(crsf_channels_config_data.block_num_next_datastruct != block){
			SD_WRITE_BLOCK(block_byte_array, 508, block);
			memset(block_byte_array, 0, sizeof(block_byte_array));
			block = crsf_channels_config_data.block_num_next_datastruct;
		}

	memcpy(&block_byte_array[index], &pid_values_config_data, sizeof(pid_values_config_data));
		index = pid_values_config_data.index_next_datastruct;
		if(pid_values_config_data.block_num_next_datastruct != block){
			SD_WRITE_BLOCK(block_byte_array, 508, block);
			memset(block_byte_array, 0, sizeof(block_byte_array));
			block = pid_values_config_data.block_num_next_datastruct;
		}

	SD_WRITE_BLOCK(block_byte_array, sizeof(block_byte_array), block);

}
