/*
 * sd_logger.c
 *
 *  Created on: May 6, 2025
 *      Author: benno
 */

#include "sd_logger.h"
#include "fatfs.h"
#include "debug.h"

static Sensor_Data* sensor_data;
static CRSF_DATA* crsf_data;
static GPS_NAV_PVT* gps_nav_pvt;

static bool armed = false;
static bool last_arm_status = false;


void SD_LOGGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT){

	sensor_data = SENSOR_DATA;
	crsf_data = CRSF_DATA;
	gps_nav_pvt = GPS_NAV_PVT;

}

void SD_LOGGER_LOOP_CALL(){
	last_arm_status = false;

	/*
	// open file at arm
	if(last_arm_status == false && armed == true){
		last_arm_status = true;
		res = f_open(&file, "/testfile.bin", FA_OPEN_ALWAYS | FA_WRITE);
		if(res != FR_OK){
			Error_Handler();
		}
		else if(res == FR_OK){
			res = f_lseek(&file, f_size(&file));
			if(res != FR_OK){
				Error_Handler();
			}
		}
		STATUS_LED_GREEN_ON();
		return;
	}
	// log file while armed
	if(armed && last_arm_status){
		unsigned int bytes_written;
		//res = f_write(&file, &crsf_data->channel[0], sizeof(crsf_data->channel[0]), &bytes_written);
		uint8_t test_buf[] = {1, 2, 3, 4, 5};
		res = f_write(&file, test_buf, sizeof(test_buf), &bytes_written);
		if(res != FR_OK){
			Error_Handler();
		}
		return;
	}
	// close file after disarm
	if(last_arm_status == true && armed == false){
		last_arm_status = false;
		res = f_sync(&file);
		if(res != FR_OK){
			Error_Handler();
		}
		res = f_close(&file);
		if(res != FR_OK){
			Error_Handler();
		}
		STATUS_LED_GREEN_OFF();
		return;
	}*/
}

void SD_LOGGER_FORWARD_ARM(bool ARM_STATUS){
	armed = ARM_STATUS;
}
