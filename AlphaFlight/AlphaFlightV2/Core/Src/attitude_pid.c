/*
 * attitude_pid.c
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#include "attitude_pid.h"
#include "math.h"
#include "utils.h"
#include "time-utils.h"
#include "debug.h"
#include "servo.h"
#include "flight_control.h"
#include "onboard-sensors.h"
#include "crossfire.h"
#include "flight_state.h"
#include <math.h>
#include "config_data.h"

extern FLY_BY_WIRE_SETPOINTS fly_by_wire_setpoints;
static CURRENT_SERVO_POINTS current_servo_points;

FLY_BY_WIRE_PID_VALUES attitude_pid_values = {0};


static void FC_PID_MIXER(float pitchDeflection, float rollDeflection, float throttle){
	// pitch and roll should be -1.0f...1.0f, throttle 0.0f...1.0f

	float servoLeft = rollDeflection + pitchDeflection;
	float servoRight = rollDeflection - pitchDeflection;
	servoLeft = UTIL_MAX_F(UTIL_MIN_F(servoLeft, 1.0f), -1.0f);
	servoRight = UTIL_MAX_F(UTIL_MIN_F(servoRight, 1.0f), -1.0f);
	current_servo_points.servo_left = servoLeft * 500 + 1500;
	current_servo_points.servo_right = servoRight * 500 + 1500;
	throttle = UTIL_MAX_F(UTIL_MIN_F(throttle, 1.0f), 0.0f);
	current_servo_points.motor = throttle * 1000 + 1000;
	SERVO_SET(0, current_servo_points.servo_left);
	if(FLIGHT_STATE_IS_ARMED() && !FLIGHT_STATE_IS_RX_LOSS()){
		SERVO_SET(1, current_servo_points.motor);
	}
	else{
		SERVO_SET(1, 1000);
	}
	SERVO_SET(2, current_servo_points.servo_right);
}

void FC_PID_DIRECT_CONTROL(){
	if(FLIGHT_STATE_IS_ARMED() && !FLIGHT_STATE_IS_RX_LOSS()){
		FC_PID_MIXER(UTIL_MIN_F(CRSF_DATA.channel_norm[CONFIG_DATA.crossfire.channels.pitch] + 10.0f, 100.0f) / 50.0f - 1, UTIL_MIN_F(CRSF_DATA.channel_norm[CONFIG_DATA.crossfire.channels.roll] + 4.0f, 100.0f) / 50.0f - 1, CRSF_DATA.channel_norm[CONFIG_DATA.crossfire.channels.throttle] / 100.0f);
	}
	else{
		FC_PID_MIXER(UTIL_MIN_F(CRSF_DATA.channel_norm[CONFIG_DATA.crossfire.channels.pitch] + 10.0f, 100.0f) / 50.0f - 1, UTIL_MIN_F(CRSF_DATA.channel_norm[CONFIG_DATA.crossfire.channels.roll] + 4.0f, 100.0f) / 50.0f - 1, 0.0f);
	}
}

void FC_PID_ANGLE_MODE(uint32_t dt){
	float dt_seconds = dt / 1000000.0;
	if(dt_seconds == 0.0f) return;

	// ===================================== Calculate Error ===================================
	attitude_pid_values.pitch_error = ONBOARD_SENSORS.gyro.gyro.y + fly_by_wire_setpoints.pitch_angular_velocity;
	attitude_pid_values.roll_error = -ONBOARD_SENSORS.gyro.gyro.x - fly_by_wire_setpoints.roll_angular_velocity;

	// ===================================== I-Term ==========================================
	attitude_pid_values.pitch_error_accumulated = UTIL_MIN_F(UTIL_MAX_F(attitude_pid_values.pitch_error_accumulated + (attitude_pid_values.pitch_error * CONFIG_DATA.pid.pitch.i * dt_seconds), -CONFIG_DATA.pid.roll.i_limit / 2.0f), CONFIG_DATA.pid.roll.i_limit / 2.0f);
	attitude_pid_values.roll_error_accumulated = UTIL_MIN_F(UTIL_MAX_F(attitude_pid_values.roll_error_accumulated + (attitude_pid_values.roll_error * CONFIG_DATA.pid.roll.i * dt_seconds), -CONFIG_DATA.pid.roll.i_limit / 2.0f), CONFIG_DATA.pid.roll.i_limit / 2.0f);

	// ====================================== P and D Term =======================================
	attitude_pid_values.pitch_pid_correction = ((attitude_pid_values.pitch_error * CONFIG_DATA.pid.pitch.p) + attitude_pid_values.pitch_error_accumulated + (((attitude_pid_values.pitch_error - attitude_pid_values.pitch_error_last) / dt_seconds) * CONFIG_DATA.pid.pitch.d)) * CONFIG_DATA.pid.pitch.multiplier;
	attitude_pid_values.roll_pid_correction = -((attitude_pid_values.roll_error * CONFIG_DATA.pid.roll.p) + attitude_pid_values.roll_error_accumulated + (((attitude_pid_values.roll_error - attitude_pid_values.roll_error_last) / dt_seconds) * CONFIG_DATA.pid.roll.d)) * CONFIG_DATA.pid.roll.multiplier;

	// ====================================== Save last error for D ========================================
	attitude_pid_values.pitch_error_last = attitude_pid_values.pitch_error;
	attitude_pid_values.roll_error_last = attitude_pid_values.roll_error;

	if(FLIGHT_STATE_IS_ARMED() && !FLIGHT_STATE_IS_RX_LOSS()){
		FC_PID_MIXER(attitude_pid_values.pitch_pid_correction, attitude_pid_values.roll_pid_correction, CRSF_DATA.channel_norm[CONFIG_DATA.crossfire.channels.throttle] / 100.0f);
	}
	else{
		FC_PID_MIXER(attitude_pid_values.pitch_pid_correction, attitude_pid_values.roll_pid_correction, 0.0f);
	}
}

void FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(uint32_t dt){
	float dt_seconds = dt / 1000000.0;
	if(dt_seconds == 0.0f) return;

	attitude_pid_values.pitch_error = ONBOARD_SENSORS.gyro.pitch_angle - fly_by_wire_setpoints.pitch_angle;
	attitude_pid_values.roll_error = ONBOARD_SENSORS.gyro.roll_angle - fly_by_wire_setpoints.roll_angle;

	//attitude_pid_values.pitch_error_accumulated = UTIL_MAX_F(UTIL_MIN_F(attitude_pid.pitch_error_accumulated + (attitude_pid.pitch_error * fbw_pid_settings.pitch_i * dt), -0.15), 0.15);
	//attitude_pid.roll_error_accumulated = UTIL_MAX_F(UTIL_MIN_F(attitude_pid.roll_error_accumulated + (attitude_pid.roll_error * fbw_pid_settings.roll_i * dt), -0.15), 0.15);
	attitude_pid_values.pitch_pid_correction = ((attitude_pid_values.pitch_error * CONFIG_DATA.pid.pitch.p) + attitude_pid_values.pitch_error_accumulated + (((attitude_pid_values.pitch_error - attitude_pid_values.pitch_error_last) / dt_seconds) * CONFIG_DATA.pid.pitch.d)) * CONFIG_DATA.pid.pitch.multiplier;
	attitude_pid_values.roll_pid_correction = -((attitude_pid_values.roll_error * CONFIG_DATA.pid.roll.p) + attitude_pid_values.roll_error_accumulated + (((attitude_pid_values.roll_error - attitude_pid_values.roll_error_last) / dt_seconds) * CONFIG_DATA.pid.roll.d)) * CONFIG_DATA.pid.roll.multiplier;
	attitude_pid_values.pitch_error_last = attitude_pid_values.pitch_error;
	attitude_pid_values.roll_error_last = attitude_pid_values.roll_error;

	if(FLIGHT_STATE_IS_ARMED() && !FLIGHT_STATE_IS_RX_LOSS()){
		FC_PID_MIXER(attitude_pid_values.pitch_pid_correction, attitude_pid_values.roll_pid_correction, CRSF_DATA.channel_norm[CONFIG_DATA.crossfire.channels.throttle] / 100.0f);
	}
	else{
		FC_PID_MIXER(attitude_pid_values.pitch_pid_correction, attitude_pid_values.roll_pid_correction, 0.0);
	}
}

void FC_PID_PRINT_CURRENT_SERVO_POINTS(){
	USB_PRINTLN("%d, %d, %d, %f", current_servo_points.servo_left, current_servo_points.motor, current_servo_points.servo_right, CRSF_DATA.channel_norm[5]);
	//USB_PRINTLN("FBW Pitch: %f, FBW Roll: %f", fly_by_wire_setpoints->pitch_angle, fly_by_wire_setpoints->roll_angle);
}
