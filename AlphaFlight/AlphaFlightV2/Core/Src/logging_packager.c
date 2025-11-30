/*
 * logging_packager.c
 *
 *  Created on: Jun 11, 2025
 *      Author: benno
 */

#include "logging_packager.h"
#include "stm32f7xx_hal.h"
#include "string.h"
#include "stdbool.h"
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

extern CRSF_DATA crsf_data;
extern GPS_NAV_PVT gps_nav_pvt;
extern FLY_BY_WIRE_PID_VALUES attitude_pid_values;
extern FLY_BY_WIRE_SETPOINTS fly_by_wire_setpoints;

static uint8_t logging_buffer[LOG_BUFFER_SIZE] = {0};
static uint8_t* logging_buffer_pointer = logging_buffer;

LOG_GYRO_GENERAL_T log_gyro_general = {0};

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t TOPIC){

	void snapshot_gyro_general(){
		log_gyro_general.header.start_magic = LOG_FRAME_START_MAGIC;
		log_gyro_general.header.log_struct_length = sizeof(log_gyro_general);
		log_gyro_general.header.log_type = LOG_TYPE_GYRO_GENERAL;
		log_gyro_general.header.log_version = LOG_VERSION;
		log_gyro_general.header.timestamp = 0;

		log_gyro_general.quaternion_values[0] = ONBOARD_SENSORS.gyro.q_angle[0];
		log_gyro_general.quaternion_values[1] = ONBOARD_SENSORS.gyro.q_angle[1];
		log_gyro_general.quaternion_values[2] = ONBOARD_SENSORS.gyro.q_angle[2];
		log_gyro_general.quaternion_values[3] = ONBOARD_SENSORS.gyro.q_angle[3];
		log_gyro_general.gyro_x_rad = ONBOARD_SENSORS.gyro.gyro.x;
		log_gyro_general.gyro_y_rad = ONBOARD_SENSORS.gyro.gyro.y;
		log_gyro_general.gyro_z_rad = ONBOARD_SENSORS.gyro.gyro.z;
		log_gyro_general.gyro_pitch_angle = ONBOARD_SENSORS.gyro.pitch_angle;
		log_gyro_general.gyro_roll_angle = ONBOARD_SENSORS.gyro.roll_angle;

		log_gyro_general.end.end_magic = LOG_FRAME_END_MAGIC;
	}

	switch (TOPIC) {
		case LOG_TYPE_DISABLE_LOGGING:		// default
			break;
		case LOG_TYPE_GYRO_GENERAL:

			snapshot_gyro_general();

			memcpy(logging_buffer_pointer + 1, &log_gyro_general, sizeof(log_gyro_general));
			logging_buffer[0] = sizeof(log_gyro_general);
			break;
		default:
			break;
	}
	return &logging_buffer[0];
}
