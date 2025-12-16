/*
 * logging_packager.h
 *
 *  Created on: Jun 11, 2025
 *      Author: benno
 */

#ifndef INC_LOGGING_PACKAGER_H_
#define INC_LOGGING_PACKAGER_H_

#include "stdint.h"

#define LOG_FRAME_BASE_MAGIC ((uint16_t)0xC85A)
#define LOG_FRAME_START_MAGIC ((uint16_t)LOG_FRAME_BASE_MAGIC)
#define LOG_FRAME_END_MAGIC ((uint16_t)~LOG_FRAME_BASE_MAGIC)
#define LOG_BUFFER_SIZE 256
#define LOG_VERSION 1

typedef enum{
	LOG_TYPE_DISABLE_LOGGING = 0,
	LOG_TYPE_ONBOARD_SENSORS = 1,
	LOG_TYPE_CRSF = 2,
	LOG_TYPE_GPS = 3,
	LOG_TYPE_PID = 4
}LOG_TYPES;

typedef struct __attribute__((packed)){
	uint16_t start_magic;
	uint8_t log_struct_length;
	uint8_t log_type;
	uint8_t log_version;
	uint32_t timestamp;
}log_general_header_t;

typedef struct __attribute__((packed)){
	uint16_t end_magic;
}log_general_end_t;

typedef struct __attribute__((packed)){
	log_general_header_t header;

	float gyro_x_rad;
	float gyro_y_rad;
	float gyro_z_rad;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_pitch_angle;
	float gyro_roll_angle;
	float gyro_quaternion_values[4];

	float baro_pressure;
	float baro_pressure_filtered;
	float baro_pressure_base;
	float baro_height;
	float baro_vertical_speed_cm_s;
	float baro_temperature;

	float vbat;
	uint32_t vbat_raw;

	log_general_end_t end;
}LOG_ONBOARD_SENSORS_T;

typedef struct __attribute__((packed)){
	log_general_header_t header;

	uint16_t channel_raw[16];
	uint32_t last_channel_update;
	uint16_t rssi;

	log_general_end_t end;
}LOG_CRSF_T;

typedef struct __attribute__((packed)){
	log_general_header_t header;

	int32_t lat;
	int32_t lon;
	float gspeed;
	float altitude;
	float heading;
	float velN;        // m/s
	float velE;        // m/s
	float velD;        // m/s
	uint8_t numSV;
	uint8_t fix_type;

	log_general_end_t end;
}LOG_GPS_T;

typedef struct __attribute__((packed)){
	log_general_header_t header;

	float pitch_error;
	float pitch_d_correction;
	float pitch_integral;
	float pitch_pid_correction;
	float pitch_setpoint;

	float roll_error;
	float roll_d_error;
	float roll_integral;
	float roll_pid_correction;
	float roll_setpoint;

	log_general_end_t end;
}LOG_PID_T;

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t MODE, uint64_t TIMESTAMP);

#endif /* INC_LOGGING_PACKAGER_H_ */
