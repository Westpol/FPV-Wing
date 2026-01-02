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
#define CONFIG_BLOCKS_RESERVED 30

#define START_MAGIC (uint32_t)0xA3D7BC49

typedef struct __attribute((packed)){
	uint32_t config_length;
	uint8_t version;
}CONFIG_HEADER_T;

typedef struct __attribute((packed)){
	int8_t x_direction;
	int8_t y_direction;
	int8_t z_direction;
	float gyro_x_filter;
	float gyro_y_filter;
	float gyro_z_filter;
	uint16_t gyro_polling_rate;
}GYRO_CONFIG_DATA_T;

typedef struct __attribute((packed)){
	uint64_t log_mode;
}SD_LOGGER_CONFIG_DATA_T;

typedef struct __attribute((packed)){
	uint8_t throttle;
	uint8_t roll;
	uint8_t pitch;
	uint8_t arm_switch;
	uint8_t mode_switch;
}CRSF_CHANNELS_T;

typedef struct __attribute((packed)){
	CRSF_CHANNELS_T channels;

	uint8_t telemetry_enabled;
}CRSF_CONFIG_DATA_T;

typedef struct __attribute((packed)){
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
}PID_VALUES_CONFIG_DATA_T;

typedef struct __attribute((packed)){
	float b[4];
	float k_i;
	float k_p;
}MAHONY_VALUES_CONFIG_DATA_T;

typedef struct __attribute((packed)){
	uint32_t magic_start;
	CONFIG_HEADER_T config_header;
	GYRO_CONFIG_DATA_T gyro_config;
	SD_LOGGER_CONFIG_DATA_T sd_logger_config;
	CRSF_CONFIG_DATA_T crsf_config;
	PID_VALUES_CONFIG_DATA_T pid_values_config;
	MAHONY_VALUES_CONFIG_DATA_T mahony_config;
	uint32_t magic_end;
}CONFIG_PACKED_T;

void LOAD_CONFIG_INIT();
void CONFIG_WRITE();
void CONFIG_READ();
uint8_t CONFIG_WAS_READ();

#endif /* INC_LOAD_CONFIG_H_ */
