/*
 * usb_manager.c
 *
 *  Created on: Oct 28, 2025
 *      Author: benno
 */

#include "usb_manager.h"
#include "flight_state.h"
#include "usbd_cdc.h"
#include "time-utils.h"
#include "main.h"
#include "debug.h"
#include <stdbool.h>
#include <string.h>

uint8_t cdc_buffer[APP_RX_DATA_SIZE] = {0};

extern USBD_HandleTypeDef hUsbDeviceFS;
TIM_HandleTypeDef* MICROS64_HTIM = NULL;

bool enter_usb_mode_next_loop = false;
bool usb_config_mode_enabled = false;

void USB_INIT(){
	MICROS64_HTIM = TIME_UTILS_GET_TIMER();
	if(MICROS64_HTIM == NULL){
		ERROR_HANDLER_BLINKS(1);
	}
	USB_CHECK_FOR_CONNECTION();
}


void USB_CHECK_FOR_CONNECTION(){
	if(!FLIGHT_STATE_IS_ARMED()){
		if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
			FLIGHT_STATE_USB_ENABLE_OVERRIDE(FLIGHT_STATE_USB_OVERRIDE_KEY);

			if(enter_usb_mode_next_loop){
				enter_usb_mode_next_loop = false;
				usb_config_mode_enabled = true;
				HAL_TIM_Base_Stop(MICROS64_HTIM);
				__HAL_TIM_DISABLE(MICROS64_HTIM);

				while(1){
					if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED){
						FLIGHT_STATE_USB_DISABLE_OVERRIDE(FLIGHT_STATE_USB_OVERRIDE_KEY);
						break;
					}
				}

				__HAL_TIM_ENABLE(MICROS64_HTIM);
				HAL_TIM_Base_Start(MICROS64_HTIM);
				usb_config_mode_enabled = false;
			}

		}
		else{
			FLIGHT_STATE_USB_DISABLE_OVERRIDE(FLIGHT_STATE_USB_OVERRIDE_KEY);
		}
	}
}

void USB_DECODE_COMMAND(uint32_t length){	// set commands / variables here
	const char expected[] = "clear";
	uint32_t expected_len = sizeof(expected) - 1;
	if(length >= expected_len){
		if(memcmp(cdc_buffer, expected, expected_len) == 0){
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS, cdc_buffer, length);
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
		}
	}
}

void USB_DECODE_MESSAGE(uint32_t length){
	if(usb_config_mode_enabled == false){
		const char expected[] = "START_COMMUNICATION";
		uint32_t expected_len = sizeof(expected) - 1;
		if(length >= expected_len){
			if(memcmp(cdc_buffer, expected, expected_len) == 0){
				USB_PRINTLN_BLOCKING("GREAT SUCCESS!");
				enter_usb_mode_next_loop = true;
			}
			else{
				USBD_CDC_SetTxBuffer(&hUsbDeviceFS, cdc_buffer, length);
				USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			}
		}
	}
	else{
		USB_DECODE_COMMAND(length);
	}
}
