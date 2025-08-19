/*
 * attitude_pid.h
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#ifndef INC_ATTITUDE_PID_H_
#define INC_ATTITUDE_PID_H_

#include "stdint.h"
#include "stdbool.h"

typedef struct{
	uint16_t servo_left;
	uint16_t servo_right;
	uint16_t motor;
}CURRENT_SERVO_POINTS;

typedef struct{
	float pitch_p;
	float pitch_i;
	float pitch_d;
	float pitch_gain;
	float roll_p;
	float roll_i;
	float roll_d;
	float roll_gain;
	float throttle_p;
	float throttle_i;
	float throttle_d;
	float throttle_gain;
}FLY_BY_WIRE_PID;

typedef struct{
	float pitch_error;		// basically P
	float roll_error;
	float pitch_error_last;	// together with error basically D
	float roll_error_last;
	float pitch_error_accumulated;		// holds I
	float roll_error_accumulated;
	float pitch_pid_correction;
	float roll_pid_correction;
}FLY_BY_WIRE_PID_VALUES;

void FC_PID_INIT();
void FC_PID_DIRECT_CONTROL(bool armed);
void FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(bool armed, uint32_t dt);

// debug
void FC_PID_PRINT_CURRENT_SERVO_POINTS();

#endif /* INC_ATTITUDE_PID_H_ */
