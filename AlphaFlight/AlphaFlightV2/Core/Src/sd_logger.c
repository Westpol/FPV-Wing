/*
 * sd_logger.c
 *
 *  Created on: May 6, 2025
 *      Author: benno
 */

#include "sd_logger.h"
#include "fatfs.h"
#include "debug.h"

static FATFS fs;
static FRESULT res;
static FIL file;

static Sensor_Data* sensor_data;
static CRSF_DATA* crsf_data;
static GPS_NAV_PVT* gps_nav_pvt;

static bool armed = false;
static bool last_arm_status = false;

#define TEST_BLOCK      10   // pick a free block (away from boot sector)
#define TIMEOUT_MS      5000

extern SD_HandleTypeDef hsd1;

__attribute__((aligned(32))) static uint8_t test_tx[512];

void SD_LOGGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT){
	/*res = f_mount(&fs, "", 1);
	if(res != FR_OK){
		Error_Handler();
	}*/

	if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
	    ERROR_HANDLER_BLINKS(5);  // Not ready
	}

	for (int i = 0; i < 512; i++) test_tx[i] = (uint8_t)i;  // fill with 0,1,2,...

	SCB_CleanDCache_by_Addr((uint32_t *)test_tx, ((512 + 31) / 32) * 32);

	if (HAL_SD_WriteBlocks(&hsd1, test_tx, TEST_BLOCK, 1, TIMEOUT_MS) != HAL_OK) {
		ERROR_HANDLER_BLINKS(2);  // write failed
	}
	// wait until finished
	uint32_t start = HAL_GetTick();
	while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
	    if (HAL_GetTick() - start > TIMEOUT_MS) ERROR_HANDLER_BLINKS(3);
	}

	sensor_data = SENSOR_DATA;
	crsf_data = CRSF_DATA;
	gps_nav_pvt = GPS_NAV_PVT;

}

void SD_LOGGER_LOOP_CALL(){
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
	}
}

void SD_LOGGER_FORWARD_ARM(bool ARM_STATUS){
	armed = ARM_STATUS;
}
