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

extern IMU_Data imu_data;
extern CRSF_DATA crsf_data;
extern GPS_NAV_PVT gps_nav_pvt;
extern FLY_BY_WIRE_PID_VALUES attitude_pid;

extern bool arm_status;
extern bool rx_lost;

static uint8_t logging_buffer[128] = {0};
static uint8_t* logging_buffer_pointer = logging_buffer;

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t MODE){
	uint8_t array_size = 0;
	switch (MODE) {
		case 0:		// default
			uint32_t time_mode_0 = HAL_GetTick();
			memcpy(logging_buffer_pointer + 1 + array_size, &time_mode_0, sizeof(time_mode_0));
			array_size += sizeof(time_mode_0);
			memcpy(logging_buffer_pointer + 1 + array_size, &crsf_data.channel[0], sizeof(crsf_data.channel[0]));
			array_size += sizeof(crsf_data.channel[0]);
			logging_buffer[0] = array_size;
			break;
		default:
			break;
	}
	return &logging_buffer[0];
}


uint32_t LOGGING_INTERVAL_MICROSECONDS(uint16_t MODE){
	switch (MODE) {
		case 0:		// default
			return 10000;
		default:
			break;
	}
	return 0;
}
