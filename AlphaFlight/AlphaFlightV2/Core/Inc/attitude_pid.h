/*
 * attitude_pid.h
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#ifndef INC_ATTITUDE_PID_H_
#define INC_ATTITUDE_PID_H_

#include "flight_control.h"
#include "onboard-sensors.h"
#include "crossfire.h"
#include "stdint.h"

typedef struct{
	uint16_t servo_left;
	uint16_t servo_right;
	uint16_t motor;
}CURRENT_SERVO_POINTS;

void FC_PID_INIT(CRSF_DATA *crsf_d, Sensor_Data *sensor_d);
void FC_PID_DIRECT_CONTROL();

// debug
void FC_PID_PRINT_CURRENT_SERVO_POINTS();

#endif /* INC_ATTITUDE_PID_H_ */
