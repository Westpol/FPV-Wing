/*
 * sd_logger.h
 *
 *  Created on: May 6, 2025
 *      Author: benno
 */

#ifndef INC_SD_LOGGER_H_
#define INC_SD_LOGGER_H_

#include "stdint.h"
#include "stdbool.h"
#include "onboard-sensors.h"
#include "crossfire.h"
#include "m10-gps.h"

void SD_LOGGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT);
void SD_LOGGER_LOOP_CALL();
void SD_LOGGER_FORWARD_ARM(bool ARM_STATUS);

#endif /* INC_SD_LOGGER_H_ */
