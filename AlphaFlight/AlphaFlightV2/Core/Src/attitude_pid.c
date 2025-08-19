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

extern FLY_BY_WIRE_SETPOINTS fly_by_wire_setpoints;
static CURRENT_SERVO_POINTS current_servo_points;
extern CRSF_DATA crsf_data;
extern IMU_Data imu_data;

FLY_BY_WIRE_PID_VALUES attitude_pid = {0};
static FLY_BY_WIRE_PID fbw_pid_settings = {0};

float angle_x_lowpass = 0;
float angle_y_lowpass = 0;

const float lowpass_alpha_x = 0.2;
const float lowpass_alpha_y = 0.2;

static void FC_PID_MIXER(float pitchDeflection, float rollDeflection, float throttle){
	float servoLeft = pitchDeflection + rollDeflection;
	float servoRight = pitchDeflection - rollDeflection;
	servoLeft = UTIL_MIN_F(UTIL_MAX_F(servoLeft, 1.0), -1.0);
	servoRight = UTIL_MIN_F(UTIL_MAX_F(servoRight, 1.0), -1.0);
	current_servo_points.servo_left = servoLeft * 500 + 1500;
	current_servo_points.servo_right = servoRight * 500 + 1500;
	current_servo_points.motor = throttle * 1000 + 1000;
	SERVO_SET(0, current_servo_points.servo_left);
	SERVO_SET(1, current_servo_points.motor);
	SERVO_SET(2, current_servo_points.servo_right);

}

void FC_PID_INIT(){
	fbw_pid_settings.pitch_d = 10.0;
	fbw_pid_settings.pitch_p = 10.0;
	fbw_pid_settings.pitch_i = 0.001;
	fbw_pid_settings.pitch_gain = 1.0;
	fbw_pid_settings.roll_d = 10.0;
	fbw_pid_settings.roll_p = 10.0;
	fbw_pid_settings.roll_i = 0.001;
	fbw_pid_settings.roll_gain = 1.0;
}

void FC_PID_DIRECT_CONTROL(){
	if(FLIGHT_STATE_IS_ARMED()){
		FC_PID_MIXER(((((float)crsf_data.channel[2] - 172.0) / 1637.0) * 2.0 - 1.0), ((((float)crsf_data.channel[1] - 172.0) / 1637.0) * 2.0 - 1.0), (((float)crsf_data.channel[0] - 172.0) / 1637.0));
	}
	else{
		FC_PID_MIXER(((((float)crsf_data.channel[2] - 172.0) / 1637.0) * 2.0 - 1.0), ((((float)crsf_data.channel[1] - 172.0) / 1637.0) * 2.0 - 1.0), 0.0);
	}
}

void FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(uint32_t dt){
	float dt_seconds = dt / 1000000.0;
	if(dt_seconds == 0.0f) return;
	angle_x_lowpass = imu_data.angle_x_fused * lowpass_alpha_x + angle_x_lowpass * (1 -lowpass_alpha_x);
	angle_y_lowpass = imu_data.angle_y_fused * lowpass_alpha_y + angle_y_lowpass * (1 -lowpass_alpha_y);
	attitude_pid.pitch_error = fly_by_wire_setpoints.pitch_angle - angle_x_lowpass;
	attitude_pid.roll_error = fly_by_wire_setpoints.roll_angle - angle_x_lowpass;
	//attitude_pid.pitch_error_accumulated = UTIL_MAX_F(UTIL_MIN_F(attitude_pid.pitch_error_accumulated + (attitude_pid.pitch_error * fbw_pid_settings.pitch_i * dt), -0.15), 0.15);
	//attitude_pid.roll_error_accumulated = UTIL_MAX_F(UTIL_MIN_F(attitude_pid.roll_error_accumulated + (attitude_pid.roll_error * fbw_pid_settings.roll_i * dt), -0.15), 0.15);
	attitude_pid.pitch_pid_correction = -((attitude_pid.pitch_error * fbw_pid_settings.pitch_p) + attitude_pid.pitch_error_accumulated + (((attitude_pid.pitch_error - attitude_pid.pitch_error_last) / dt_seconds) * fbw_pid_settings.pitch_d)) * fbw_pid_settings.pitch_gain;
	attitude_pid.roll_pid_correction = -((attitude_pid.roll_error * fbw_pid_settings.roll_p) + attitude_pid.roll_error_accumulated + (((attitude_pid.roll_error - attitude_pid.roll_error_last) / dt_seconds) * fbw_pid_settings.roll_d)) * fbw_pid_settings.roll_gain;
	attitude_pid.pitch_error_last = attitude_pid.pitch_error;
	attitude_pid.roll_error_last = attitude_pid.roll_error;

	if(FLIGHT_STATE_IS_ARMED()){
		FC_PID_MIXER(attitude_pid.pitch_pid_correction, attitude_pid.roll_pid_correction, (((float)crsf_data.channel[0] - 172.0) / 1637.0));
	}
	else{
		FC_PID_MIXER(attitude_pid.pitch_pid_correction, attitude_pid.roll_pid_correction, 0.0);
	}
}

void FC_PID_PRINT_CURRENT_SERVO_POINTS(){
	USB_PRINTLN("%d, %d, %d, %d", current_servo_points.servo_left, current_servo_points.motor, current_servo_points.servo_right, crsf_data.channel[5]);
	//USB_PRINTLN("FBW Pitch: %f, FBW Roll: %f", fly_by_wire_setpoints->pitch_angle, fly_by_wire_setpoints->roll_angle);
}
