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
#include <stdbool.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
TIM_HandleTypeDef* MICROS64_HTIM = NULL;

bool enter_usb_mode_next_loop = false;

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
			}

		}
		else{
			FLIGHT_STATE_USB_DISABLE_OVERRIDE(FLIGHT_STATE_USB_OVERRIDE_KEY);
		}
	}
}
