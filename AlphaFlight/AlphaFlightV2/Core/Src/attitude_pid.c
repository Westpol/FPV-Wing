/*
 * attitude_pid.c
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#include "attitude_pid.h"
#include "math.h"
#include "utils.h"
#include "debug.h"

static FLY_BY_WIRE_SETPOINTS *fly_by_wire_setpoints;
static CURRENT_SERVO_POINTS current_servo_points;
static CRSF_DATA *crsf_data;
static Sensor_Data *sensor_data;

static void FC_PID_MIXER(float pitchDeflection, float rollDeflection, float throttle){
	float servoLeft = pitchDeflection + rollDeflection;
	float servoRight = pitchDeflection - rollDeflection;
	servoLeft = MIN(MAX(servoLeft, 1.0), -1.0);
	servoRight = MIN(MAX(servoRight, 1.0), -1.0);
	current_servo_points.servo_left = servoLeft * 500 + 1500;
	current_servo_points.servo_right = servoRight * 500 + 1500;
	current_servo_points.motor = throttle * 1000 + 1000;
	SERVO_SET(0, current_servo_points.servo_left);
	SERVO_SET(1, current_servo_points.motor);
	SERVO_SET(2, current_servo_points.servo_right);

}

void FC_PID_INIT(CRSF_DATA *crsf_d, Sensor_Data *sensor_d){
	crsf_data = crsf_d;
	sensor_data = sensor_d;
}

void FC_PID_DIRECT_CONTROL(){
	FC_PID_MIXER(((((float)crsf_data->channel[2] - 172.0) / 1637.0) * 2.0 - 1.0), ((((float)crsf_data->channel[1] - 172.0) / 1637.0) * 2.0 - 1.0), (((float)crsf_data->channel[0] - 172.0) / 1637.0));
}

void FC_PID_PRINT_CURRENT_SERVO_POINTS(){
	USB_PRINTLN("%d, %d, %d, %d", current_servo_points.servo_left, current_servo_points.motor, current_servo_points.servo_right, crsf_data->channel[5]);
}
