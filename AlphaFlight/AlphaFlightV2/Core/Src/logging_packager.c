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

#define SET_FLAG_COND(flags, bit, cond) ((flags) = ((flags) & ~(1U << (bit))) | ((!!(cond)) << (bit)))

extern IMU_Data imu_data;
extern CRSF_DATA crsf_data;
extern GPS_NAV_PVT gps_nav_pvt;
extern GPS_DATA gps_data;
extern FLY_BY_WIRE_PID_VALUES attitude_pid;

extern bool arm_status;
extern bool rx_lost;

static uint8_t logging_buffer[BUFFER_SIZE] = {0};
static uint8_t* logging_buffer_pointer = logging_buffer;

T1V0_GENERAL_DATA t1v0_general_data = {0};

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t MODE){

	void snapshot_t1v0_general(){
		t1v0_general_data.start_magic = LOG_FRAME_START_MAGIC;
		t1v0_general_data.end_magic = LOG_FRAME_END_MAGIC;
		t1v0_general_data.timestamp = MICROS64();
		t1v0_general_data.baro_altimeter = imu_data.height;
		t1v0_general_data.crsf_ch[0] = crsf_data.channel[0];
		t1v0_general_data.crsf_ch[1] = crsf_data.channel[1];
		t1v0_general_data.crsf_ch[2] = crsf_data.channel[2];
		t1v0_general_data.crsf_ch[3] = crsf_data.channel[3];
		t1v0_general_data.gps_heading = gps_data.heading;
		t1v0_general_data.gps_height = gps_data.height;
		t1v0_general_data.gps_lat = gps_data.lat;
		t1v0_general_data.gps_lon = gps_data.lon;
		t1v0_general_data.gps_sats = gps_nav_pvt.numSV;
		t1v0_general_data.gps_speed = gps_data.gspeed;
		t1v0_general_data.angle_fused_x = imu_data.angle_x_fused;
		t1v0_general_data.angle_fused_y = imu_data.angle_y_fused;
		t1v0_general_data.angle_fused_z = imu_data.angle_z_fused;

		SET_FLAG_COND(t1v0_general_data.status_flags, 0, arm_status);
		SET_FLAG_COND(t1v0_general_data.status_flags, 1, rx_lost);
	}

	switch (MODE) {
		case LOG_TYPE_DISABLE_LOGGING:		// default
			break;
		case LOG_TYPE_T1V0_GENERAL:

			snapshot_t1v0_general();

			memcpy(logging_buffer_pointer + 1, &t1v0_general_data, sizeof(t1v0_general_data));
			logging_buffer[0] = sizeof(t1v0_general_data);
			break;
		default:
			break;
	}
	return &logging_buffer[0];
}


uint32_t LOGGING_PACKER_INTERVAL_MICROSECONDS(uint16_t MODE){

	static const uint32_t log_intervals_us[] = {
		[LOG_TYPE_DISABLE_LOGGING] = HZ_TO_DELTA_T_US(10),
		[LOG_TYPE_T1V0_GENERAL]    = HZ_TO_DELTA_T_US(50),
	};

	return log_intervals_us[MODE];
}
