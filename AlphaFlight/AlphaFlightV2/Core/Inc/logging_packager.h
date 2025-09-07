/*
 * logging_packager.h
 *
 *  Created on: Jun 11, 2025
 *      Author: benno
 */

#ifndef INC_LOGGING_PACKAGER_H_
#define INC_LOGGING_PACKAGER_H_

#include "stdint.h"

#define LOG_FRAME_START_MAGIC 0xC8
#define LOG_FRAME_END_MAGIC 0x9A
#define BUFFER_SIZE 254

typedef enum{
	LOG_TYPE_DISABLE_LOGGING = 0,
	LOG_TYPE_T1V0_GENERAL = 1,
	LOG_TYPE_THROTTLE = 2,
	LOG_TYPE_GYRO = 3,
	LOG_TYPE_SENSORS = 4,
	LOG_TYPE_FLY_BY_WIRE = 5
}LOG_TYPES;

typedef struct __attribute__((packed)){
	uint16_t start_magic;

	uint64_t timestamp;

	float roll_angle, pitch_angle;		// onboard-sensors

	float baro_altimeter;

	double gps_lon, gps_lat, gps_height, gps_speed, gps_heading;		// GPS
	uint8_t gps_sats;

	uint16_t crsf_ch[4];		// user inputs

	uint16_t status_flags;

	float pid_correction_roll, pid_correction_pitch;
	float fbw_setpoint_roll, fbw_setpoint_pitch;

	uint16_t end_magic;

}T1V0_GENERAL_DATA;

uint8_t* LOGGING_PACKER_BY_MODE(uint16_t MODE);

uint32_t LOGGING_PACKER_INTERVAL_MICROSECONDS(uint16_t MODE);

#endif /* INC_LOGGING_PACKAGER_H_ */
