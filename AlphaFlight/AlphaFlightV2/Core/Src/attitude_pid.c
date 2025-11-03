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
extern CRSF_DATA crsf_data;
extern IMU_Data imu_data;
extern float q[4];

FLY_BY_WIRE_PID_VALUES attitude_pid = {0};
FLY_BY_WIRE_PID fbw_pid_settings = {0};

float angle_x_lowpass = 0;
float angle_y_lowpass = 0;

const float lowpass_alpha_x = 0.2;
const float lowpass_alpha_y = 0.2;

static void FC_PID_MIXER(float pitchDeflection, float rollDeflection, float throttle){
	// pitch and roll should be -1.0f...1.0f, throttle 0.0f...1.0f

	float servoLeft = rollDeflection + pitchDeflection;
	float servoRight = rollDeflection - pitchDeflection;
	servoLeft = UTIL_MIN_F(UTIL_MAX_F(servoLeft, 1.0f), -1.0f);
	servoRight = UTIL_MIN_F(UTIL_MAX_F(servoRight, 1.0f), -1.0f);
	current_servo_points.servo_left = servoLeft * 500 + 1500;
	current_servo_points.servo_right = servoRight * 500 + 1500;
	throttle = UTIL_MIN_F(UTIL_MAX_F(throttle, 1.0f), 0.0f);
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

void FC_PID_INIT(){
	fbw_pid_settings.pitch_d = 20.0;
	fbw_pid_settings.pitch_p = 20.0;
	fbw_pid_settings.pitch_i = 0.001;
	fbw_pid_settings.pitch_gain = 1.0;
	fbw_pid_settings.roll_d = 20.0;
	fbw_pid_settings.roll_p = 20.0;
	fbw_pid_settings.roll_i = 0.001;
	fbw_pid_settings.roll_gain = 1.0;
}

void FC_PID_DIRECT_CONTROL(){
	if(FLIGHT_STATE_IS_ARMED() && !FLIGHT_STATE_IS_RX_LOSS()){
		FC_PID_MIXER(UTIL_MAX_F(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.pitch] + 10.0f, 100.0f) / 50.0f - 1, UTIL_MAX_F(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.roll] + 4.0f, 100.0f) / 50.0f - 1, crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.throttle] / 100.0f);
	}
	else{
		FC_PID_MIXER(UTIL_MAX_F(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.pitch] + 10.0f, 100.0f) / 50.0f - 1, UTIL_MAX_F(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.roll] + 4.0f, 100.0f) / 50.0f - 1, 0.0);
	}
}

void FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(uint32_t dt){
	float dt_seconds = dt / 1000000.0;
	if(dt_seconds == 0.0f) return;

	float pitch_rad = -fly_by_wire_setpoints.pitch_angle * M_PI / 180.0f;	// in radians
	float roll_rad  = fly_by_wire_setpoints.roll_angle  * M_PI / 180.0f;	// in radians

	float q_setpoint[4] = {0};
	float q_setpoint_pitch[4] = {cosf(pitch_rad / 2.0f), 0, sinf(pitch_rad / 2.0f), 0};
	float q_setpoint_roll[4] = {cosf(roll_rad / 2.0f), sinf(roll_rad / 2.0f), 0, 0};
	float q_inverted[4] = {q[0], -q[1], -q[2], -q[3]};
	float q_error[4];

	UTIL_QUATERNION_PRODUCT(q_setpoint_pitch, q_setpoint_roll, q_setpoint);
	UTIL_QUATERNION_PRODUCT(q_setpoint, q_inverted, q_error);

	if(q_error[0] < 0){
		q_error[0] = -q_error[0];
		q_error[1] = -q_error[1];
		q_error[2] = -q_error[2];
		q_error[3] = -q_error[3];
	}

	float angle_total = 2 * acosf(q_error[0]);
	float scalar = sqrtf(1 - q_error[0] * q_error[0]);

	if(scalar != 0){
		attitude_pid.pitch_error = (q_error[2] / scalar) * angle_total;
		attitude_pid.roll_error = (q_error[1] / scalar) * angle_total;
	}
	else{
		attitude_pid.pitch_error = 0;
		attitude_pid.roll_error = 0;
	}

	//attitude_pid.pitch_error_accumulated = UTIL_MAX_F(UTIL_MIN_F(attitude_pid.pitch_error_accumulated + (attitude_pid.pitch_error * fbw_pid_settings.pitch_i * dt), -0.15), 0.15);
	//attitude_pid.roll_error_accumulated = UTIL_MAX_F(UTIL_MIN_F(attitude_pid.roll_error_accumulated + (attitude_pid.roll_error * fbw_pid_settings.roll_i * dt), -0.15), 0.15);
	attitude_pid.pitch_pid_correction = ((attitude_pid.pitch_error * fbw_pid_settings.pitch_p) + attitude_pid.pitch_error_accumulated + (((attitude_pid.pitch_error - attitude_pid.pitch_error_last) / dt_seconds) * fbw_pid_settings.pitch_d)) * fbw_pid_settings.pitch_gain;
	attitude_pid.roll_pid_correction = -((attitude_pid.roll_error * fbw_pid_settings.roll_p) + attitude_pid.roll_error_accumulated + (((attitude_pid.roll_error - attitude_pid.roll_error_last) / dt_seconds) * fbw_pid_settings.roll_d)) * fbw_pid_settings.roll_gain;
	attitude_pid.pitch_error_last = attitude_pid.pitch_error;
	attitude_pid.roll_error_last = attitude_pid.roll_error;

	if(FLIGHT_STATE_IS_ARMED() && !FLIGHT_STATE_IS_RX_LOSS()){
		FC_PID_MIXER(attitude_pid.pitch_pid_correction, attitude_pid.roll_pid_correction, crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.throttle] / 100.0f);
	}
	else{
		FC_PID_MIXER(attitude_pid.pitch_pid_correction, attitude_pid.roll_pid_correction, 0.0);
	}
}

void FC_PID_PRINT_CURRENT_SERVO_POINTS(){
	USB_PRINTLN("%d, %d, %d, %f", current_servo_points.servo_left, current_servo_points.motor, current_servo_points.servo_right, crsf_data.channel_norm[5]);
	//USB_PRINTLN("FBW Pitch: %f, FBW Roll: %f", fly_by_wire_setpoints->pitch_angle, fly_by_wire_setpoints->roll_angle);
}
