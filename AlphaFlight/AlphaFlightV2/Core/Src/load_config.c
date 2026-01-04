/*
 * load_config.c
 *
 *  Created on: Oct 8, 2025
 *      Author: benno
 */

#include "load_config.h"

#include <stdint.h>
#include <string.h>

#include "sd.h"

#include "debug.h"
#include "main.h"
#include "config_data.h"
#include "logging_packager.h"
#include "utils.h"

CONFIG_PACKED_T config_packed;

uint8_t config_been_read = 0;

/*static uint32_t next_magic(uint32_t prev) {
    uint32_t x = prev;
    x ^= (x << 13);
    x ^= (x >> 17);
    x ^= (x << 5);
    return x | 0x80000000; // ensure non-zero and high bit set
}*/


#ifdef CONFIG_WRITE_STANDARD_ENABLED
static void CONFIG_SET_STANDARD_VALUES(){
	if(config_been_read == 0){
		config_packed.magic_start = START_MAGIC;
		config_packed.magic_end = ~START_MAGIC;

		config_packed.config_header.config_length = sizeof(config_packed);
		config_packed.config_header.version = CONFIG_VERSION;


		config_packed.sd_logger_config.log_mode = (1 << 0) | (1 << LOG_TYPE_ONBOARD_SENSORS) | (1 << LOG_TYPE_CRSF) | (1 << LOG_TYPE_GPS) | (1 << LOG_TYPE_PID);


		config_packed.crsf_config.channels.throttle = 0;
		config_packed.crsf_config.channels.roll = 2;
		config_packed.crsf_config.channels.pitch = 1;
		config_packed.crsf_config.channels.arm_switch = 11;
		config_packed.crsf_config.channels.mode_switch = 5;

		config_packed.crsf_config.telemetry_enabled = 1;


		config_packed.pid_values_config.pitch_p = 1;
		config_packed.pid_values_config.pitch_i = 0.5;
		config_packed.pid_values_config.pitch_d = 2;
		config_packed.pid_values_config.pitch_d_filter = 1;
		config_packed.pid_values_config.pitch_feed_forward = 1;
		config_packed.pid_values_config.pitch_i_limit = 0.3;
		config_packed.pid_values_config.pitch_i_zone = 0.3;
		config_packed.pid_values_config.pitch_multiplier = 1;

		config_packed.pid_values_config.roll_p = 1;
		config_packed.pid_values_config.roll_i = 0.5;
		config_packed.pid_values_config.roll_d = 2;
		config_packed.pid_values_config.roll_d_filter = 1;
		config_packed.pid_values_config.roll_feed_forward = 1;
		config_packed.pid_values_config.roll_i_limit = 0.3;
		config_packed.pid_values_config.roll_i_zone = 0.3;
		config_packed.pid_values_config.roll_multiplier = 1;


		config_packed.mahony_config.b[0] = 0;
		config_packed.mahony_config.b[1] = 0;
		config_packed.mahony_config.b[2] = 0;
		config_packed.mahony_config.k_i = -0.05;
		config_packed.mahony_config.k_p = 0.5;
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
	CONFIG_DATA.logger.log_mode = config_packed.sd_logger_config.log_mode;

	CONFIG_DATA.crossfire.channels.throttle = config_packed.crsf_config.channels.throttle;
	CONFIG_DATA.crossfire.channels.roll = config_packed.crsf_config.channels.roll;
	CONFIG_DATA.crossfire.channels.pitch = config_packed.crsf_config.channels.pitch;
	CONFIG_DATA.crossfire.channels.arm_switch = config_packed.crsf_config.channels.arm_switch;
	CONFIG_DATA.crossfire.channels.mode_switch = config_packed.crsf_config.channels.mode_switch;
	CONFIG_DATA.crossfire.telemetry.enabled = config_packed.crsf_config.telemetry_enabled;

	CONFIG_DATA.pid.roll.d_filter = config_packed.pid_values_config.roll_d_filter;
	CONFIG_DATA.pid.roll.feed_forward = config_packed.pid_values_config.roll_feed_forward;
	CONFIG_DATA.pid.roll.i = config_packed.pid_values_config.roll_i;
	CONFIG_DATA.pid.roll.i_limit = config_packed.pid_values_config.roll_i_limit;
	CONFIG_DATA.pid.roll.i_zone = config_packed.pid_values_config.roll_i_zone;
	CONFIG_DATA.pid.roll.multiplier = config_packed.pid_values_config.roll_multiplier;
	CONFIG_DATA.pid.roll.p = config_packed.pid_values_config.roll_p;

	CONFIG_DATA.pid.pitch.d_filter = config_packed.pid_values_config.pitch_d_filter;
	CONFIG_DATA.pid.pitch.feed_forward = config_packed.pid_values_config.pitch_feed_forward;
	CONFIG_DATA.pid.pitch.i = config_packed.pid_values_config.pitch_i;
	CONFIG_DATA.pid.pitch.i_limit = config_packed.pid_values_config.pitch_i_limit;
	CONFIG_DATA.pid.pitch.i_zone = config_packed.pid_values_config.pitch_i_zone;
	CONFIG_DATA.pid.pitch.multiplier = config_packed.pid_values_config.pitch_multiplier;
	CONFIG_DATA.pid.pitch.p = config_packed.pid_values_config.pitch_p;

	CONFIG_DATA.mahony.b[0] = config_packed.mahony_config.b[0];
	CONFIG_DATA.mahony.b[1] = config_packed.mahony_config.b[1];
	CONFIG_DATA.mahony.b[2] = config_packed.mahony_config.b[2];
	CONFIG_DATA.mahony.k_i = config_packed.mahony_config.k_i;
	CONFIG_DATA.mahony.k_p = config_packed.mahony_config.k_p;
	CONFIG_DATA_BACKUP_DATA();
}

void CONFIG_READ(){
	uint8_t block_data[BLOCK_DATA_SIZE];
	uint32_t struct_size_left = sizeof(config_packed);
	uint16_t copy_length;

	for(int i = 0; i < CONFIG_BLOCKS_RESERVED; i++){
		if(struct_size_left <= 0) break;	// if less or equal zero, complete struct is copied to

		SD_READ_BLOCK(block_data, CONFIG_START_BLOCK + i);

		if(struct_size_left > BLOCK_DATA_SIZE){
			copy_length = BLOCK_DATA_SIZE;
			struct_size_left -= BLOCK_DATA_SIZE;
		}
		else{
			copy_length = struct_size_left;
			struct_size_left -= copy_length;
		}

		memcpy(((uint8_t *)&config_packed) + i * BLOCK_DATA_SIZE, block_data, copy_length);		// copy buffer data to struct at offset
	}

	if(config_packed.magic_start != START_MAGIC || config_packed.magic_end != ~START_MAGIC) ERROR_HANDLER_BLINKS(6);
	if(config_packed.config_header.version != CONFIG_VERSION) ERROR_HANDLER_BLINKS(7);

	CONFIG_SET_CONFIG_DATA_STRUCT();
	config_been_read = 1;
}



void CONFIG_WRITE(){
	int32_t struct_size_left = config_packed.config_header.config_length;
	uint16_t copy_length = 0;
	uint8_t block_byte_array[BLOCK_DATA_SIZE] = {0};

	if(struct_size_left > CONFIG_BLOCKS_RESERVED * BLOCK_DATA_SIZE) ERROR_HANDLER_BLINKS(5);	// check for too big struct for reserved space

	for(int i = 0; i < CONFIG_BLOCKS_RESERVED; i++){
		if(struct_size_left <= 0) break;	// if less or equal zero, complete struct is copied

		if(struct_size_left > BLOCK_DATA_SIZE){
			copy_length = BLOCK_DATA_SIZE;
			struct_size_left -= BLOCK_DATA_SIZE;
		}
		else{
			copy_length = struct_size_left;
			struct_size_left -= copy_length;
			memset(block_byte_array, 0, BLOCK_DATA_SIZE);
		}
		memcpy(block_byte_array, ((uint8_t *)&config_packed) + i * BLOCK_DATA_SIZE, copy_length);		// copy struct data to buffer at offset
		SD_WRITE_BLOCK(block_byte_array, BLOCK_DATA_SIZE, i + CONFIG_START_BLOCK);
	}
}
