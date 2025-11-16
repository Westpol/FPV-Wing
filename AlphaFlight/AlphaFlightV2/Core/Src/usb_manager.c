/*
 * usb_manager.c
 *
 *  Created on: Oct 28, 2025
 *      Author: benno
 */

#include "usb_manager.h"
#include <stdbool.h>
#include <string.h>

#include "flight_state.h"
#include "usbd_cdc.h"
#include "time-utils.h"
#include "main.h"
#include "debug.h"

uint8_t cdc_buffer[APP_RX_DATA_SIZE] = {0};
static uint8_t clear_buffer[64];

extern USBD_HandleTypeDef hUsbDeviceFS;
TIM_HandleTypeDef* MICROS64_HTIM = NULL;

bool enter_usb_mode_next_loop = false;
bool usb_config_mode_enabled = false;
bool exit_command_line_config = false;

typedef struct{
	uint8_t mode_point;
	uint8_t parting_indexes[4];
	uint8_t strings[5][32];

	uint8_t command;
	uint8_t topic;
	uint8_t topic_value_1;
	uint8_t topic_value_2;
	uint8_t value;
}message_decoder_t;

message_decoder_t message_decoder = {0};

void USB_INIT(){
	MICROS64_HTIM = TIME_UTILS_GET_TIMER();
	if(MICROS64_HTIM == NULL){
		ERROR_HANDLER_BLINKS(1);
	}
	USB_CHECK_FOR_CONNECTION();
	for(int i = 0; i < 64;i++){
		clear_buffer[i] = (i % 2 == 0)? 13 : 10;
	}
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
					if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || exit_command_line_config){
						exit_command_line_config = false;
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

	memset(&message_decoder, 0, sizeof(message_decoder_t));

	for(int i = 0; i < length; i++){
		if('A' <= cdc_buffer[i] && 'Z' >= cdc_buffer[i]){		// changing upper case to lower case, valid char for now
			cdc_buffer[i] += 'a' - 'A';
			continue;
		}

		if('0' <= cdc_buffer[i] && '9' >= cdc_buffer[i]) continue;		// valid char for now

		if(cdc_buffer[i] == ' '){
			if(message_decoder.mode_point == 0){
				if(i >= 31) return;		// change to length of longest command later

				for(int f = 0; f < i; f++){
					message_decoder.strings[0][f] = cdc_buffer[f];
				}
				message_decoder.strings[0][i] = '\0';		// make shure that null termination is correct
				message_decoder.parting_indexes[message_decoder.mode_point] = i;
				message_decoder.mode_point = 1;
			}
			else{
				if(message_decoder.mode_point >= 4) return;
				uint8_t last_index_space = message_decoder.parting_indexes[message_decoder.mode_point - 1] + 1;
				uint8_t current_index_space = i;
				uint8_t token_length = current_index_space - last_index_space;		// absoliute length of current token

				if(token_length >= 31) return;

				for(int f = last_index_space; f < current_index_space; f++){
					message_decoder.strings[message_decoder.mode_point][f - last_index_space] = cdc_buffer[f];
				}
				message_decoder.strings[message_decoder.mode_point][token_length] = '\0';		// make shure that null termination is correct
				message_decoder.parting_indexes[message_decoder.mode_point] = current_index_space;
				message_decoder.mode_point += 1;
			}
			continue;
		}

		if(cdc_buffer[i] == '\n'){
			if(message_decoder.mode_point == 0){
				if(i >= 31) return;		// change to length of longest command later

				for(int f = 0; f < i; f++){
					message_decoder.strings[0][f] = cdc_buffer[f];
				}
				message_decoder.strings[0][i] = '\0';		// make shure that null termination is correct
				message_decoder.parting_indexes[message_decoder.mode_point] = i;
				message_decoder.mode_point = 1;
			}
			else{
				if(message_decoder.mode_point >= 5) return;

				uint8_t last_index_space = message_decoder.parting_indexes[message_decoder.mode_point - 1] + 1;
				uint8_t current_index_space = i;
				uint8_t token_length = current_index_space - last_index_space;		// absoliute length of current token

				if(token_length >= 31) return;

				for(int f = last_index_space; f < current_index_space; f++){
					message_decoder.strings[message_decoder.mode_point][f - last_index_space] = cdc_buffer[f];
				}
				message_decoder.strings[message_decoder.mode_point][token_length] = '\0';		// make shure that null termination is correct
				message_decoder.mode_point += 1;

				break;
			}
		}

		return;		// something invalid found
	}


	const char expected[] = "clear";
	uint32_t expected_len = sizeof(expected) - 1;
	if(length >= expected_len){
		if(memcmp(cdc_buffer, expected, expected_len) == 0){
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS, clear_buffer, 64);
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
		}
	}

	const char expected_exit[] = "exit";
	uint32_t expected_len_exit = sizeof(expected_exit) - 1;
	if(length >= expected_len_exit){
		if(memcmp(cdc_buffer, expected_exit, expected_len_exit) == 0){
			USB_PRINTLN_BLOCKING("exiting command line configuration...");
			usb_config_mode_enabled = false;
			exit_command_line_config = true;
			return;
		}
	}
}

void USB_DECODE_MESSAGE(uint32_t length){		// gets called every time usb message is recieved
	if(usb_config_mode_enabled == false){		// if USB configurator is not connected
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
	else{		// if USB configurator is connected
		USB_DECODE_COMMAND(length);
	}
}
