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

static FLY_BY_WIRE_SETPOINTS *fly_by_wire_setpoints;
static CURRENT_SERVO_POINTS current_servo_points;
extern CRSF_DATA crsf_data;
extern IMU_Data imu_data;

static FLY_BY_WIRE_PID_VALUES fbw_pid = {0};
static FLY_BY_WIRE_PID fbw_pid_settings = {0};

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

void FC_PID_INIT(){
	fly_by_wire_setpoints = FC_GET_SETPOINTS();
	fbw_pid_settings.pitch_d = 0.01;
	fbw_pid_settings.pitch_p = 0.01;
	fbw_pid_settings.pitch_i = 0.001;
	fbw_pid_settings.pitch_gain = 0.01;
	fbw_pid_settings.roll_d = 0.01;
	fbw_pid_settings.roll_p = 0.01;
	fbw_pid_settings.roll_i = 0.001;
	fbw_pid_settings.roll_gain = 0.01;
}

void FC_PID_DIRECT_CONTROL(bool armed){
	if(armed){
		FC_PID_MIXER(((((float)crsf_data.channel[2] - 172.0) / 1637.0) * 2.0 - 1.0), ((((float)crsf_data.channel[1] - 172.0) / 1637.0) * 2.0 - 1.0), (((float)crsf_data.channel[0] - 172.0) / 1637.0));
	}
	else{
		FC_PID_MIXER(((((float)crsf_data.channel[2] - 172.0) / 1637.0) * 2.0 - 1.0), ((((float)crsf_data.channel[1] - 172.0) / 1637.0) * 2.0 - 1.0), 0.0);
	}
}

void FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(bool armed, uint32_t dt){
	fbw_pid.pitch_error = fly_by_wire_setpoints->pitch_angle - imu_data.angle_x_fused;
	fbw_pid.roll_error = fly_by_wire_setpoints->roll_angle - imu_data.angle_y_fused;
	fbw_pid.pitch_error_accumulated = MAX(MIN(fbw_pid.pitch_error_accumulated + (fbw_pid.pitch_error * fbw_pid_settings.pitch_i * dt), -0.15), 0.15);
	fbw_pid.roll_error_accumulated = MAX(MIN(fbw_pid.roll_error_accumulated + (fbw_pid.roll_error * fbw_pid_settings.roll_i * dt), -0.15), 0.15);
	fbw_pid.pitch_pid_correction = -((fbw_pid.pitch_error * fbw_pid_settings.pitch_p) + fbw_pid.pitch_error_accumulated + (((fbw_pid.pitch_error - fbw_pid.pitch_error_last) / dt) * fbw_pid_settings.pitch_d)) + fbw_pid_settings.pitch_gain;
	fbw_pid.roll_pid_correction = -((fbw_pid.roll_error * fbw_pid_settings.roll_p) + fbw_pid.roll_error_accumulated + (((fbw_pid.roll_error - fbw_pid.roll_error_last) / dt) * fbw_pid_settings.roll_d)) + fbw_pid_settings.roll_gain;
	fbw_pid.pitch_error_last = fbw_pid.pitch_error;
	fbw_pid.roll_error_last = fbw_pid.roll_error;

	if(armed){
		FC_PID_MIXER(fbw_pid.pitch_pid_correction, fbw_pid.roll_pid_correction, (((float)crsf_data.channel[0] - 172.0) / 1637.0));
	}
	else{
		FC_PID_MIXER(fbw_pid.pitch_pid_correction, fbw_pid.roll_pid_correction, 0.0);
	}
}

void FC_PID_PRINT_CURRENT_SERVO_POINTS(){
	USB_PRINTLN("%d, %d, %d, %d", current_servo_points.servo_left, current_servo_points.motor, current_servo_points.servo_right, crsf_data.channel[5]);
	//USB_PRINTLN("FBW Pitch: %f, FBW Roll: %f", fly_by_wire_setpoints->pitch_angle, fly_by_wire_setpoints->roll_angle);
}
