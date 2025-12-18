/*
 * load_config.h
 *
 *  Created on: Oct 8, 2025
 *      Author: benno
 */

#ifndef INC_LOAD_CONFIG_H_
#define INC_LOAD_CONFIG_H_

#include <stdint.h>

#define CONFIG_WRITE_STANDARD_ENABLED 1
#define CONFIG_START_BLOCK 60

#define START_MAGIC 0xA3D7BC49

typedef struct __attribute((packed)){
	uint32_t magic_start;
	uint16_t index_next_datastruct;
	uint32_t block_num_next_datastruct;
	uint16_t num_config_blocks;
	uint8_t version;
	uint32_t magic_end;
}CONFIG_HEADER;

typedef struct __attribute((packed)){
	uint32_t magic_start;
	uint16_t index_next_datastruct;		// sizeof(GYRO_CONFIG)
	uint32_t block_num_next_datastruct;

	int8_t x_direction;
	int8_t y_direction;
	int8_t z_direction;
	float gyro_x_filter;
	float gyro_y_filter;
	float gyro_z_filter;
	uint16_t gyro_polling_rate;

	uint32_t magic_end;
}GYRO_CONFIG_DATA;

typedef struct __attribute((packed)){
	uint32_t magic_start;
	uint16_t index_next_datastruct;
	uint32_t block_num_next_datastruct;

	uint64_t log_mode;

	uint32_t magic_end;
}SD_LOGGER_CONFIG_DATA;

typedef struct __attribute((packed)){
	uint32_t magic_start;
	uint16_t index_next_datastruct;
	uint32_t block_num_next_datastruct;

	uint8_t throttle;
	uint8_t roll;
	uint8_t pitch;
	uint8_t arm_switch;
	uint8_t mode_switch;

	uint8_t telemetry_enabled;

	uint32_t magic_end;
}CRSF_CHANNELS_CONFIG_DATA;

typedef struct __attribute((packed)){
	uint32_t magic_start;
	uint16_t index_next_datastruct;
	uint32_t block_num_next_datastruct;

	float roll_p;
	float roll_i;
	float roll_i_limit;
	float roll_i_zone;
	float roll_d;
	float roll_d_filter;
	float roll_multiplier;
	float roll_feed_forward;

	float pitch_p;
	float pitch_i;
	float pitch_i_limit;
	float pitch_i_zone;
	float pitch_d;
	float pitch_d_filter;
	float pitch_multiplier;
	float pitch_feed_forward;

	uint32_t magic_end;
}PID_VALUES_CONFIG_DATA;

typedef struct __attribute((packed)){
	uint32_t magic_start;
	uint16_t index_next_datastruct;
	uint32_t block_num_next_datastruct;

	float b[4];
	float k_i;
	float k_p;

	uint32_t magic_end;
}MAHONY_VALUES_CONFIG_DATA;

void LOAD_CONFIG_INIT();
void CONFIG_WRITE();
void CONFIG_READ();
uint8_t CONFIG_WAS_READ();

#endif /* INC_LOAD_CONFIG_H_ */
