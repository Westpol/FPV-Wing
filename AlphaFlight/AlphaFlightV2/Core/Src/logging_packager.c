/*
 * logging_packager.c
 *
 *  Created on: Jun 11, 2025
 *      Author: benno
 */

#include "logging_packager.h"
#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include "onboard-sensors.h"
#include "crossfire.h"
#include "m10-gps.h"
#include "attitude_pid.h"
#include "flight_control.h"
#include "scheduler.h"
#include "time-utils.h"
#include "flight_state.h"
#include "utils.h"

#define SET_FLAG_COND(flags, bit, cond) ((flags) = ((flags) & ~(1U << (bit))) | ((!!(cond)) << (bit)))

extern FLY_BY_WIRE_SETPOINTS fly_by_wire_setpoints;
extern FLY_BY_WIRE_PID_VALUES attitude_pid_values;

static uint8_t logging_buffer[LOG_BUFFER_SIZE] = {0};

LOG_ONBOARD_SENSORS_T log_onboard_sensors = {0};
LOG_CRSF_T log_crsf = {0};
LOG_GPS_T log_gps = {0};
LOG_PID_T log_pid = {0};

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t TOPIC, uint64_t TIMESTAMP){

	void snapshot_onboard_sensors(uint64_t timestamp){
		log_onboard_sensors.header.start_magic = LOG_FRAME_START_MAGIC;
		log_onboard_sensors.header.log_struct_length = sizeof(log_onboard_sensors);
		log_onboard_sensors.header.log_type = LOG_TYPE_ONBOARD_SENSORS;
		log_onboard_sensors.header.log_version = LOG_VERSION;
		log_onboard_sensors.header.timestamp = (uint32_t)timestamp;

		log_onboard_sensors.gyro_quaternion_values[0] = ONBOARD_SENSORS.gyro.q_angle[0];
		log_onboard_sensors.gyro_quaternion_values[1] = ONBOARD_SENSORS.gyro.q_angle[1];
		log_onboard_sensors.gyro_quaternion_values[2] = ONBOARD_SENSORS.gyro.q_angle[2];
		log_onboard_sensors.gyro_quaternion_values[3] = ONBOARD_SENSORS.gyro.q_angle[3];
		log_onboard_sensors.gyro_x_rad = ONBOARD_SENSORS.gyro.gyro.x;
		log_onboard_sensors.gyro_y_rad = ONBOARD_SENSORS.gyro.gyro.y;
		log_onboard_sensors.gyro_z_rad = ONBOARD_SENSORS.gyro.gyro.z;
		log_onboard_sensors.gyro_pitch_angle = ONBOARD_SENSORS.gyro.pitch_angle;
		log_onboard_sensors.gyro_roll_angle = ONBOARD_SENSORS.gyro.roll_angle;

		log_onboard_sensors.accel_x = ONBOARD_SENSORS.accel.accel.x;
		log_onboard_sensors.accel_y = ONBOARD_SENSORS.accel.accel.y;
		log_onboard_sensors.accel_z = ONBOARD_SENSORS.accel.accel.z;

		log_onboard_sensors.baro_height = ONBOARD_SENSORS.barometer.height;
		log_onboard_sensors.baro_pressure = ONBOARD_SENSORS.barometer.pressure;
		log_onboard_sensors.baro_pressure_base = ONBOARD_SENSORS.barometer.pressure_base;
		log_onboard_sensors.baro_pressure_filtered = ONBOARD_SENSORS.barometer.pressure_filtered;
		log_onboard_sensors.baro_temperature = ONBOARD_SENSORS.barometer.temperature;
		log_onboard_sensors.baro_vertical_speed_cm_s = ONBOARD_SENSORS.barometer.vertical_speed_cm_s;

		log_onboard_sensors.vbat = ONBOARD_SENSORS.vbat.vbat;
		log_onboard_sensors.vbat_raw = ONBOARD_SENSORS.vbat.vbat_raw;

		log_onboard_sensors.end.end_magic = LOG_FRAME_END_MAGIC;
	}

	void snapshot_crsf(uint64_t timestamp){
		log_crsf.header.start_magic = LOG_FRAME_START_MAGIC;
		log_crsf.header.log_struct_length = sizeof(log_crsf);
		log_crsf.header.log_type = LOG_TYPE_CRSF;
		log_crsf.header.log_version = LOG_VERSION;
		log_crsf.header.timestamp = (uint32_t)timestamp;

		for(int i = 0; i < 16; i++){
			log_crsf.channel_raw[i] = CRSF_DATA.channel_raw[i];
		}
		log_crsf.last_channel_update = (uint32_t)CRSF_DATA.last_channel_update;
		log_crsf.rssi = CRSF_DATA.rssi;

		log_crsf.end.end_magic = LOG_FRAME_END_MAGIC;
	}

	void snapshot_gps(uint64_t timestamp){
			log_gps.header.start_magic = LOG_FRAME_START_MAGIC;
			log_gps.header.log_struct_length = sizeof(log_gps);
			log_gps.header.log_type = LOG_TYPE_GPS;
			log_gps.header.log_version = LOG_VERSION;
			log_gps.header.timestamp = (uint32_t)timestamp;

			log_gps.lat = GPS_DATA.lat_int;
			log_gps.lon = GPS_DATA.lon_int;
			log_gps.altitude = GPS_DATA.altitude;
			log_gps.fix_type = GPS_DATA.fix_type;
			log_gps.gspeed = GPS_DATA.gspeed;
			log_gps.heading = GPS_DATA.heading;
			log_gps.numSV = GPS_DATA.numSV;
			log_gps.velD = GPS_DATA.velD;
			log_gps.velN = GPS_DATA.velN;
			log_gps.velE = GPS_DATA.velE;

			log_gps.end.end_magic = LOG_FRAME_END_MAGIC;
		}

	void snapshot_pid(uint64_t timestamp){
			log_pid.header.start_magic = LOG_FRAME_START_MAGIC;
			log_pid.header.log_struct_length = sizeof(log_pid);
			log_pid.header.log_type = LOG_TYPE_PID;
			log_pid.header.log_version = LOG_VERSION;
			log_pid.header.timestamp = (uint32_t)timestamp;

			log_pid.pitch_error = attitude_pid_values.pitch_error;
			log_pid.pitch_integral = attitude_pid_values.pitch_error_accumulated;
			log_pid.pitch_pid_correction = attitude_pid_values.pitch_pid_correction;
			log_pid.pitch_setpoint = fly_by_wire_setpoints.pitch_angular_velocity;

			log_pid.roll_error = attitude_pid_values.roll_error;
			log_pid.roll_integral = attitude_pid_values.roll_error_accumulated;
			log_pid.roll_pid_correction = attitude_pid_values.roll_pid_correction;
			log_pid.roll_setpoint = fly_by_wire_setpoints.roll_angular_velocity;

			log_pid.end.end_magic = LOG_FRAME_END_MAGIC;
		}

	switch (TOPIC) {
		case LOG_TYPE_DISABLE_LOGGING:		// default
			break;
		case LOG_TYPE_ONBOARD_SENSORS:
			snapshot_onboard_sensors(TIMESTAMP);

			memcpy(&logging_buffer[1], &log_onboard_sensors, sizeof(log_onboard_sensors));
			logging_buffer[0] = sizeof(log_onboard_sensors);
			break;

		case LOG_TYPE_CRSF:
			snapshot_crsf(TIMESTAMP);

			memcpy(&logging_buffer[1], &log_crsf, sizeof(log_crsf));
			logging_buffer[0] = sizeof(log_crsf);

			break;
		case LOG_TYPE_GPS:
			snapshot_gps(TIMESTAMP);

			memcpy(&logging_buffer[1], &log_gps, sizeof(log_gps));
			logging_buffer[0] = sizeof(log_gps);
			break;
		case LOG_TYPE_PID:
			snapshot_pid(TIMESTAMP);

			memcpy(&logging_buffer[1], &log_pid, sizeof(log_pid));
			logging_buffer[0] = sizeof(log_pid);
			break;
		default:
			break;
	}
	return &logging_buffer[0];
}
