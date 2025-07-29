/*
 * logging_packager.h
 *
 *  Created on: Jun 11, 2025
 *      Author: benno
 */

#ifndef INC_LOGGING_PACKAGER_H_
#define INC_LOGGING_PACKAGER_H_

#include "stdint.h"

typedef enum{
	LOG_TYPE_DISABLE_LOGGING = 0,
	LOG_TYPE_GENERAL,
	LOG_TYPE_THROTTLE,
	LOG_TYPE_GYRO,
	LOG_TYPE_SENSORS,
	LOG_TYPE_FLY_BY_WIRE
}LOG_TYPES;

typedef struct __attribute__((packed)){
	uint32_t timestamp;

	float gyro_fused_x, gyro_fused_y, gyro_fused_z;

	float baro_altimeter;

	int32_t gps_lon, gps_lat, gps_height, gps_speed, gps_heading;
	uint8_t gps_sats;

	uint16_t crsf_ch[4];

	uint16_t status_flags;

}T1V0_GENERAL_DATA;

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t MODE);

uint32_t LOGGING_INTERVAL_MICROSECONDS(uint16_t MODE);

#endif /* INC_LOGGING_PACKAGER_H_ */
