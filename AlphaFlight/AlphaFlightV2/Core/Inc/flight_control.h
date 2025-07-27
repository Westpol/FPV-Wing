/*
 * flight_control.h
 *
 *  Created on: Apr 17, 2025
 *      Author: benno
 */

#ifndef INC_FLIGHT_CONTROL_H_
#define INC_FLIGHT_CONTROL_H_

#include "stm32f7xx_hal.h"
#include "onboard-sensors.h"
#include "crossfire.h"

typedef enum{
	DIRECT_CONTROL,
	DIRECT_CONTROL_WITH_LIMITS,
	FLY_BY_WIRE,
	AUTOPILOT,
	RETURN_TO_HOME,
	RETURN_TO_HOME_NO_GPS
}FLIGHT_MODE;

typedef struct{
	float pitch_angle;
	float roll_angle;
	float speed_knots;
	uint32_t last_update_us;
}FLY_BY_WIRE_SETPOINTS;

void FC_SANITY_CHECK();
void FC_MODE_CHECK();
void FC_PROCESS();
FLY_BY_WIRE_SETPOINTS *FC_GET_SETPOINTS();

#endif /* INC_FLIGHT_CONTROL_H_ */
