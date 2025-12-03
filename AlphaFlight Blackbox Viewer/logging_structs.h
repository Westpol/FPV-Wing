#ifndef LOGGING_STRUCT_H
#define LOGGING_STRUCT_H

#include <stdint.h>

typedef enum{
	LOG_TYPE_DISABLE_LOGGING = 0,
	LOG_TYPE_ONBOARD_SENSORS = 1,
	LOG_TYPE_CRSF = 2,
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
	uint64_t last_channel_update;
	uint16_t rssi;

	log_general_end_t end;
}LOG_CRSF_T;

#endif