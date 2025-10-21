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
		FC_PID_MIXER(UTIL_MAX_F(crsf_data.channel_norm[CRSF_CHANNEL_PITCH] + 10.0f, 100.0f) / 50.0f - 1, UTIL_MAX_F(crsf_data.channel_norm[CRSF_CHANNEL_ROLL] + 4.0f, 100.0f) / 50.0f - 1, crsf_data.channel_norm[CRSF_CHANNEL_THROTTLE] / 100.0f);
	}
	else{
		FC_PID_MIXER(UTIL_MAX_F(crsf_data.channel_norm[CRSF_CHANNEL_PITCH] + 10.0f, 100.0f) / 50.0f - 1, UTIL_MAX_F(crsf_data.channel_norm[CRSF_CHANNEL_ROLL] + 4.0f, 100.0f) / 50.0f - 1, 0.0);
	}
}

void FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(uint32_t dt){
	float dt_seconds = dt / 1000000.0;
	if(dt_seconds == 0.0f) return;

	float pitch_rad = -fly_by_wire_setpoints.pitch_angle * M_PI / 180.0f;
	float roll_rad  = fly_by_wire_setpoints.roll_angle  * M_PI / 180.0f;

	float half_pitch = pitch_rad * 0.5f;
	float half_roll  = roll_rad  * 0.5f;

	// Assuming yaw = 0 for simplicity
	float cp = cosf(half_pitch);
	float sp = sinf(half_pitch);
	float cr = cosf(half_roll);
	float sr = sinf(half_roll);

	// q = q_pitch * q_roll  (Z-Y-X sequence, yaw=0)
	float q_target[4];

	q_target[0] = cp * cr;        // w
	q_target[1] = cp * sr;        // x
	q_target[2] = sp * cr;        // y
	q_target[3] = -sp * sr;       // z

	// optional: normalize to avoid drift
	float norm_q_target = sqrtf(q_target[0]*q_target[0] + q_target[1]*q_target[1]
	                  + q_target[2]*q_target[2] + q_target[3]*q_target[3]);
	if (norm_q_target > 0.0f) {
	    q_target[0] /= norm_q_target;
	    q_target[1] /= norm_q_target;
	    q_target[2] /= norm_q_target;
	    q_target[3] /= norm_q_target;
	}

	float qc[4] = {q[0], -q[1], -q[2], -q[3]}; // conjugate of current

	float q_err[4] = {0};

	q_err[0] = q_target[0]*qc[0] - q_target[1]*qc[1] - q_target[2]*qc[2] - q_target[3]*qc[3];
	q_err[1] = q_target[0]*qc[1] + q_target[1]*qc[0] + q_target[2]*qc[3] - q_target[3]*qc[2];
	q_err[2] = q_target[0]*qc[2] - q_target[1]*qc[3] + q_target[2]*qc[0] + q_target[3]*qc[1];
	q_err[3] = q_target[0]*qc[3] + q_target[1]*qc[2] - q_target[2]*qc[1] + q_target[3]*qc[0];

	float norm_q_err = sqrtf(q_err[0]*q_err[0] + q_err[1]*q_err[1] + q_err[2]*q_err[2] + q_err[3]*q_err[3]);

	if(norm_q_err > 0.0f){
		q_err[0] /= norm_q_err;
		q_err[1] /= norm_q_err;
		q_err[2] /= norm_q_err;
		q_err[3] /= norm_q_err;
	}

	float roll_error  = 2.0f * q_err[1];  // rotation about body X
	float pitch_error = 2.0f * q_err[2];  // rotation about body Y

	attitude_pid.pitch_error = pitch_error;
	attitude_pid.roll_error = roll_error;

	//attitude_pid.pitch_error_accumulated = UTIL_MAX_F(UTIL_MIN_F(attitude_pid.pitch_error_accumulated + (attitude_pid.pitch_error * fbw_pid_settings.pitch_i * dt), -0.15), 0.15);
	//attitude_pid.roll_error_accumulated = UTIL_MAX_F(UTIL_MIN_F(attitude_pid.roll_error_accumulated + (attitude_pid.roll_error * fbw_pid_settings.roll_i * dt), -0.15), 0.15);
	attitude_pid.pitch_pid_correction = ((attitude_pid.pitch_error * fbw_pid_settings.pitch_p) + attitude_pid.pitch_error_accumulated + (((attitude_pid.pitch_error - attitude_pid.pitch_error_last) / dt_seconds) * fbw_pid_settings.pitch_d)) * fbw_pid_settings.pitch_gain;
	attitude_pid.roll_pid_correction = -((attitude_pid.roll_error * fbw_pid_settings.roll_p) + attitude_pid.roll_error_accumulated + (((attitude_pid.roll_error - attitude_pid.roll_error_last) / dt_seconds) * fbw_pid_settings.roll_d)) * fbw_pid_settings.roll_gain;
	attitude_pid.pitch_error_last = attitude_pid.pitch_error;
	attitude_pid.roll_error_last = attitude_pid.roll_error;

	if(FLIGHT_STATE_IS_ARMED() && !FLIGHT_STATE_IS_RX_LOSS()){
		FC_PID_MIXER(attitude_pid.pitch_pid_correction, attitude_pid.roll_pid_correction, crsf_data.channel_norm[CRSF_CHANNEL_THROTTLE] / 100.0f);
	}
	else{
		FC_PID_MIXER(attitude_pid.pitch_pid_correction, attitude_pid.roll_pid_correction, 0.0);
	}
}

void FC_PID_PRINT_CURRENT_SERVO_POINTS(){
	USB_PRINTLN("%d, %d, %d, %f", current_servo_points.servo_left, current_servo_points.motor, current_servo_points.servo_right, crsf_data.channel_norm[5]);
	//USB_PRINTLN("FBW Pitch: %f, FBW Roll: %f", fly_by_wire_setpoints->pitch_angle, fly_by_wire_setpoints->roll_angle);
}
