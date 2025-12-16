#ifndef LOGGING_STRUCT_H
#define LOGGING_STRUCT_H

#include <stdint.h>
#include <stdio.h>

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

typedef struct{
	uint8_t id;
	uint8_t (*decode)(const uint8_t* raw, uint8_t mode, FILE *file);
} DECODER_T;

uint8_t copy_struct_onboard_sensors(const uint8_t* raw, uint8_t mode, FILE *file);

uint8_t copy_struct_crsf(const uint8_t* raw, uint8_t mode, FILE *file);

uint8_t copy_struct_gps(const uint8_t* raw, uint8_t mode, FILE *file);

uint8_t copy_struct_pid(const uint8_t* raw, uint8_t mode, FILE *file);

#endif