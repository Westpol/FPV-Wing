/*
 * logging_packager.c
 *
 *  Created on: Jun 11, 2025
 *      Author: benno
 */

#include "logging_packager.h"
#include "stm32f7xx_hal.h"
#include "string.h"
#include "main.h"

static Sensor_Data* sensor_data;
static CRSF_DATA* crsf_data;
static GPS_NAV_PVT* gps_nav_pvt;

static uint8_t logging_buffer[128] = {0};
static uint8_t* logging_buffer_pointer = logging_buffer;

void LOGGING_PACKAGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT){
	sensor_data = SENSOR_DATA;
	crsf_data = CRSF_DATA;
	gps_nav_pvt = GPS_NAV_PVT;
}

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t MODE){
	switch (MODE) {
		case 0:		// default
			uint32_t time_mode_0 = HAL_GetTick();
			memcpy(logging_buffer_pointer + 1, &time_mode_0, sizeof(time_mode_0));
			logging_buffer[0] = sizeof(time_mode_0);
			break;
		default:
			break;
	}
	return &logging_buffer[0];
}


uint32_t LOGGING_INTERVAL_MICROSECONDS(uint16_t MODE){
	switch (MODE) {
		case 0:		// default
			return 50000;
			break;
		default:
			break;
	}
	return 0;
}
