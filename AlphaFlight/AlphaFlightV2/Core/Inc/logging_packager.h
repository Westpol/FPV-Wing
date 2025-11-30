/*
 * logging_packager.h
 *
 *  Created on: Jun 11, 2025
 *      Author: benno
 */

#ifndef INC_LOGGING_PACKAGER_H_
#define INC_LOGGING_PACKAGER_H_

#include "stdint.h"

#define LOG_FRAME_BASE_MAGIC 0xC85A
#define LOG_FRAME_START_MAGIC LOG_FRAME_BASE_MAGIC
#define LOG_FRAME_END_MAGIC ~LOG_FRAME_BASE_MAGIC
#define LOG_BUFFER_SIZE 256
#define LOG_VERSION 1

typedef enum{
	LOG_TYPE_DISABLE_LOGGING = 0,
	LOG_TYPE_GYRO_GENERAL = 1,
}LOG_TYPES;

typedef struct __attribute__((packed)){
	uint32_t start_magic;
	uint8_t log_struct_length;
	uint8_t log_type;
	uint8_t log_version;
	uint64_t timestamp;
}log_general_header_t;

typedef struct __attribute__((packed)){
	uint32_t end_magic;
}log_general_end_t;

typedef struct __attribute__((packed)){
	log_general_header_t header;
	float gyro_x_rad;
	float gyro_y_rad;
	float gyro_z_rad;
	float gyro_pitch_angle;
	float gyro_roll_angle;
	float quaternion_values[4];
	log_general_end_t end;
}LOG_GYRO_GENERAL_T;

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t MODE);

#endif /* INC_LOGGING_PACKAGER_H_ */
