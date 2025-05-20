/*
 * sd_logger.c
 *
 *  Created on: May 6, 2025
 *      Author: benno
 */

#include "sd_logger.h"
#include "debug.h"
#include "stm32f7xx_hal.h"

static Sensor_Data* sensor_data;
static CRSF_DATA* crsf_data;
static GPS_NAV_PVT* gps_nav_pvt;

static bool armed = false;
static bool last_arm_status = false;


extern SD_HandleTypeDef hsd1;

#define BLOCK_SIZE     512
#define BLOCK_NUMBER   10000
#define TIMEOUT_MS     1000

// Needs to be 32-byte aligned due to D-Cache
__attribute__((aligned(32))) uint8_t tx_buffer[BLOCK_SIZE];


void SD_LOGGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT){

	// 1. Fill data buffer
	    for (int i = 0; i < BLOCK_SIZE; i++) {
	        tx_buffer[i] = i & 0xFF;
	    }

	    // 2. Clean D-Cache before DMA access
	    SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer, ((BLOCK_SIZE + 31) / 32) * 32);

	    // 3. Optional: Check card state
	    if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
	        ERROR_HANDLER_BLINKS(1); // Not ready
	    }

	    // check init
	    if (HAL_SD_Init(&hsd1) != HAL_OK) {
	        ERROR_HANDLER_BLINKS(4);  // Init failed
	    }

	    // 4. Write block
	    if (HAL_SD_WriteBlocks(&hsd1, tx_buffer, BLOCK_NUMBER, 1, TIMEOUT_MS) != HAL_OK) {
	        ERROR_HANDLER_BLINKS(2); // Write failed
	    }

	    // 5. Wait until the card is ready again
	    uint32_t start = HAL_GetTick();
	    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
	        if (HAL_GetTick() - start > TIMEOUT_MS) {
	            ERROR_HANDLER_BLINKS(3); // Timeout
	        }
	    }

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
