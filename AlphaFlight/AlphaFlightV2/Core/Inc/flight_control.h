/*
 * flight_control.h
 *
 *  Created on: Apr 17, 2025
 *      Author: benno
 */

#ifndef INC_FLIGHT_CONTROL_H_
#define INC_FLIGHT_CONTROL_H_

#include "stm32f7xx_hal.h"

typedef enum{
	DIRECT_CONTROL,
	DIRECT_CONTROL_WITH_LIMITS,
	ANGLE_MODE,
	FLY_BY_WIRE,
	AUTOPILOT,
	RETURN_TO_HOME,
	RETURN_TO_HOME_NO_GPS
}FLIGHT_MODE;

typedef struct{
	float pitch_angle;
	float roll_angle;
	float pitch_angular_velocity;
	float roll_angular_velocity;
	float speed_knots;
	uint32_t last_update_us;
}FLY_BY_WIRE_SETPOINTS;

void FC_SANITY_CHECK();
void FC_MODE_CHECK();
void FC_PROCESS();

#endif /* INC_FLIGHT_CONTROL_H_ */
