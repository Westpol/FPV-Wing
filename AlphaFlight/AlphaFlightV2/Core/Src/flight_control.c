/*
 * flight_control.c
 *
 *  Created on: Apr 17, 2025
 *      Author: benno
 */

#include "flight_control.h"
#include "time-utils.h"
#include "utils.h"
#include "stdbool.h"
#include "attitude_pid.h"
#include "debug.h"
#include "sd_logger.h"
#include "m10-gps.h"
#include "onboard-sensors.h"
#include "crossfire.h"
#include "flight_state.h"
#include "config_data.h"

extern GPS_NAV_PVT gps_nav_pvt;
extern CRSF_DATA crsf_data;
static FLIGHT_MODE current_flight_mode = DIRECT_CONTROL;
FLY_BY_WIRE_SETPOINTS fly_by_wire_setpoints = {0};
static bool arm_failed = false;

static uint64_t last_process_execution_time = 0;

void FC_SANITY_CHECK(){
	if(crsf_data.last_channel_update + 5000000 < MICROS64()){
		FLIGHT_STATE_RX_LOSS(FLIGHT_STATE_RXLOSS_CHANGE_KEY);
	}
	else if(FLIGHT_STATE_IS_RX_LOSS()){
		FLIGHT_STATE_RX_VALID(FLIGHT_STATE_RXLOSS_CHANGE_KEY);
	}
	/*if(gps_nav_pvt.numSV < 5){	TODO: Implement rxloss fallback logic
		if(current_flight_mode == AUTOPILOT && !rx_lost){
			current_flight_mode = FLY_BY_WIRE;
		}
		else if(rx_lost){
			current_flight_mode = RETURN_TO_HOME_NO_GPS;
		}
	}*/
}

void FC_MODE_CHECK(){
	if(!FLIGHT_STATE_IS_RX_LOSS()){
		if(crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.arm_switch] > 60 && FLIGHT_STATE_IS_ARMED() == false && arm_failed == false){
			if(crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.throttle] < 1){
				FLIGHT_STATE_ARM(FLIGHT_STATE_ARM_CHANGE_KEY);
				FLIGHT_STATE_SET_EVENT(FLIGHT_STATE_ARMED);
			}
			else{
				arm_failed = true;
				FLIGHT_STATE_SET_EVENT(FLIGHT_STATE_ARM_FAILED);
			}
		}
		if((crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.arm_switch] < 60 && FLIGHT_STATE_IS_ARMED()) || (crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.arm_switch] < 60 && arm_failed == true)){
			FLIGHT_STATE_DISARM(FLIGHT_STATE_ARM_CHANGE_KEY);
			arm_failed = false;
			FLIGHT_STATE_SET_EVENT(FLIGHT_STATE_DISARMED);
		}

		if(crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.mode_switch] < 10){
			current_flight_mode = DIRECT_CONTROL;
			fly_by_wire_setpoints.roll_angle = ONBOARD_SENSORS.gyro.roll_angle;
			fly_by_wire_setpoints.pitch_angle = ONBOARD_SENSORS.gyro.pitch_angle;
		}
		else if(crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.mode_switch] < 60){
			current_flight_mode = ANGLE_MODE;
		}
		else{
			//current_flight_mode = AUTOPILOT;
			current_flight_mode = FLY_BY_WIRE;
		}
	}
}

static float FC_CRSF_DEADBAND(float value, float deadband_width, float deadband_point){
	if(value > (deadband_point - deadband_width) && value < (deadband_point + deadband_width)){
		return deadband_point;
	}
	return value;
}

void FC_PROCESS(){
	uint64_t dt = MICROS64() - last_process_execution_time;
	last_process_execution_time = MICROS64();

	if(dt < 100000){	// skip setpoint set if deltaT is too big
		switch(current_flight_mode){
		case FLY_BY_WIRE:
			fly_by_wire_setpoints.pitch_angle = UTIL_MAX_F(UTIL_MIN_F(fly_by_wire_setpoints.pitch_angle - (((FC_CRSF_DEADBAND(crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.pitch], 0.5, 50) / 50.0f - 1) * UTIL_RADIANS(10)) / (1000000.0f / dt)), UTIL_RADIANS(30)), UTIL_RADIANS(-25));
			fly_by_wire_setpoints.roll_angle = UTIL_MAX_F(UTIL_MIN_F(fly_by_wire_setpoints.roll_angle - (((FC_CRSF_DEADBAND(crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.roll], 0.5, 50) / 50.0f - 1) * UTIL_RADIANS(15)) / (1000000.0f / dt)), UTIL_RADIANS(45)), UTIL_RADIANS(-45));
			break;
		case ANGLE_MODE:
			fly_by_wire_setpoints.pitch_angular_velocity = (FC_CRSF_DEADBAND(crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.pitch], 0.5, 50) / 50.0f - 1) * UTIL_RADIANS(20);
			fly_by_wire_setpoints.roll_angular_velocity = (FC_CRSF_DEADBAND(crsf_data.channel_norm[CONFIG_DATA.crossfire.channels.roll], 0.5, 50) / 50.0f - 1) * UTIL_RADIANS(30);
			break;
		default:
			break;
		}
	}

	switch (current_flight_mode) {
		case DIRECT_CONTROL:
			FC_PID_DIRECT_CONTROL();
			break;
		case ANGLE_MODE:
			FC_PID_ANGLE_MODE(dt);
			break;
		case FLY_BY_WIRE:
			FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(dt);
			break;
		default:
			break;
	}
}
