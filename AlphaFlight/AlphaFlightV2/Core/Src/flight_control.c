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

extern IMU_Data imu_data;
extern GPS_NAV_PVT gps_nav_pvt;
extern CRSF_DATA crsf_data;
static FLIGHT_MODE current_flight_mode = DIRECT_CONTROL;
FLY_BY_WIRE_SETPOINTS fly_by_wire_setpoints = {0};
bool rx_lost = false;
bool arm_status = false;
static bool arm_failed = false;

static uint64_t last_process_execution_time = 0;

void FC_SANITY_CHECK(){
	if(crsf_data.last_channel_update + 5000000 < MICROS64()){
		rx_lost = true;
	}
	else if(rx_lost){
		rx_lost = false;
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
	if(!rx_lost){
		if(crsf_data.channel[11] > 1000 && arm_status == false && arm_failed == false){
			if(crsf_data.channel[0] < 200){
				arm_status = true;
			}
			else{
				arm_failed = true;
			}
		}
		if((crsf_data.channel[11] < 1000 && arm_status == true) || (crsf_data.channel[11] < 1000 && arm_failed == true)){
			arm_status = false;
			arm_failed = false;
		}

		if(crsf_data.channel[5] < 500){
			current_flight_mode = DIRECT_CONTROL;
			fly_by_wire_setpoints.roll_angle = imu_data.angle_x_fused;
			fly_by_wire_setpoints.pitch_angle = imu_data.angle_y_fused;
		}
		else if(crsf_data.channel[5] < 1500){
			current_flight_mode = FLY_BY_WIRE;
		}
		else{
			current_flight_mode = AUTOPILOT;
		}
	}
}

void FC_PROCESS(){
	uint64_t dt = MICROS64() - last_process_execution_time;
	last_process_execution_time = MICROS64();
	if(dt > 100000){
		return;
	}

	fly_by_wire_setpoints.pitch_angle = UTIL_MIN_F(UTIL_MAX_F(fly_by_wire_setpoints.pitch_angle - ((((((float)crsf_data.channel[2] - 172.0) / 1637.0) * 2.0 - 1.0) * 10) / (1000000 / dt)), 30), -25);
	fly_by_wire_setpoints.roll_angle = UTIL_MIN_F(UTIL_MAX_F(fly_by_wire_setpoints.roll_angle - ((((((float)crsf_data.channel[1] - 172.0) / 1637.0) * 2.0 - 1.0) * 15) / (1000000 / dt)), 45), -45);

	switch (current_flight_mode) {
		case DIRECT_CONTROL:
			FC_PID_DIRECT_CONTROL(arm_status);
			break;
		case DIRECT_CONTROL_WITH_LIMITS:
			break;
		case FLY_BY_WIRE:
			//STATUS_LED_GREEN_ON();
			FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(arm_status, dt);
			//STATUS_LED_GREEN_OFF();
			break;
		default:
			break;
	}
}
