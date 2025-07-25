/*
 * logging_packager.h
 *
 *  Created on: Jun 11, 2025
 *      Author: benno
 */

#ifndef INC_LOGGING_PACKAGER_H_
#define INC_LOGGING_PACKAGER_H_

#include "stdint.h"
#include "onboard-sensors.h"
#include "crossfire.h"
#include "m10-gps.h"

typedef enum{
	LOG_TYPE_DISABLE_LOGGING = 0,
	LOG_TYPE_GENERAL,
	LOG_TYPE_THROTTLE,
	LOG_TYPE_GYRO,
	LOG_TYPE_SENSORS,
	LOG_TYPE_FLY_BY_WIRE
}LOG_TYPES;

void LOGGING_PACKAGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT);

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t MODE);

uint32_t LOGGING_INTERVAL_MICROSECONDS(uint16_t MODE);

#endif /* INC_LOGGING_PACKAGER_H_ */
