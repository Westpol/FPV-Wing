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
#include "load_config.h"
#include "cli_get_set.h"

uint8_t cdc_buffer[APP_RX_DATA_SIZE] = {0};
static uint8_t clear_buffer[64];

extern USBD_HandleTypeDef hUsbDeviceFS;

bool enter_usb_mode_next_loop = false;
bool usb_config_mode_enabled = false;
bool exit_command_line_config = false;

typedef struct{
	uint8_t mode_point;
	uint8_t parting_indexes[4];
	char strings[5][32];

	uint8_t command;
	uint8_t topic;
	uint8_t topic_value_1;
	uint8_t topic_value_2;
	uint8_t value;
}message_decoder_t;

message_decoder_t message_decoder = {0};

void USB_INIT(){
	USB_CHECK_FOR_CONNECTION();
	for(int i = 0; i < 64;i++){
		clear_buffer[i] = (i % 2 == 0)? 13 : 10;
	}
}

void USB_DECODE_COMMAND(uint32_t length){	// set commands / variables here

	memset(&message_decoder, 0, sizeof(message_decoder_t));

	for(int i = 0; i < length; i++){
		if('A' <= cdc_buffer[i] && 'Z' >= cdc_buffer[i]){		// changing upper case to lower case, valid char for now
			cdc_buffer[i] += 'a' - 'A';
			continue;
		}

		if('a' <= cdc_buffer[i] && 'z' >= cdc_buffer[i]) continue;

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
				message_decoder.strings[0][i] = '\0';		// make sure that null termination is correct
				message_decoder.parting_indexes[message_decoder.mode_point] = i;
				message_decoder.mode_point = 1;
				break;
			}
			else{
				if(message_decoder.mode_point >= 5) return;

				uint8_t last_index_space = message_decoder.parting_indexes[message_decoder.mode_point - 1] + 1;
				uint8_t current_index_space = i;
				uint8_t token_length = current_index_space - last_index_space;		// absolute length of current token

				if(token_length >= 31) return;

				for(int f = last_index_space; f < current_index_space; f++){
					message_decoder.strings[message_decoder.mode_point][f - last_index_space] = cdc_buffer[f];
				}
				message_decoder.strings[message_decoder.mode_point][token_length] = '\0';		// make sure that null termination is correct
				message_decoder.mode_point += 1;

				break;
			}
		}

		return;		// something invalid found
	}

	enum{
		SIMPLE_COMMAND_CLEAR = 0,
		SIMPLE_COMMAND_EXIT = 1,
		SIMPLE_COMMAND_SAVE = 2,
		SIMPLE_COMMAND_DUMP = 3
	};

	if(message_decoder.mode_point == 1){		// single command (save, exit, clear, dump)

		uint8_t basic_commands = 4;
		const char *command[] = {"clear", "exit", "save", "dump"};

		for(int i = 0; i < basic_commands; i++){

			if(strcmp((const char *)message_decoder.strings[0], command[i]) != 0) continue;

			switch (i) {
				case SIMPLE_COMMAND_CLEAR:
					USBD_CDC_SetTxBuffer(&hUsbDeviceFS, clear_buffer, 64);
					USBD_CDC_TransmitPacket(&hUsbDeviceFS);
					break;
				case SIMPLE_COMMAND_EXIT:
					USB_PRINTLN_BLOCKING("exiting command line configuration...");
					usb_config_mode_enabled = false;
					exit_command_line_config = true;
					return;
					break;
				case SIMPLE_COMMAND_SAVE:
					CONFIG_WRITE();
					break;
				default:
					break;
			}
		}
	}

	if(message_decoder.mode_point == 2){
		uint8_t basic_commands = 1;
		const char *command[] = {"get"};

		for(int i = 0; i < basic_commands; i++){
			if(strcmp((const char*)message_decoder.strings[0], command[i]) != 0) continue;

			switch(i){
			case 0:
				if(!CLI_PROCESS_GET_COMMAND(message_decoder.strings[1])){
					USB_PRINTLN("Command not found!");
				}
				break;
			default:
				break;
			}
		}
	}
}


void USB_DECODE_MESSAGE(uint32_t length){		// gets called every time usb message is recieved
	if(usb_config_mode_enabled == false){		// if USB configurator is not connected / enabled
		const char expected[] = "start";
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
	else{		// if USB configurator is connected / enabled
		USB_DECODE_COMMAND(length);
	}
}


void USB_CHECK_FOR_CONNECTION(){		// logic while USB connection is established
	if(!FLIGHT_STATE_IS_ARMED()){
		if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
			FLIGHT_STATE_USB_ENABLE_OVERRIDE(FLIGHT_STATE_USB_OVERRIDE_KEY);

			if(enter_usb_mode_next_loop){
				enter_usb_mode_next_loop = false;
				usb_config_mode_enabled = true;
				TIME_UTILS_PAUSE_MICROS64();
				bool usb_activated = true;

				// USB mode global variables here

				uint64_t led_delay = 500 * 1000;

				// USB mode global variables here

				// USB Mode default config

				STATUS_LED_BLUE_OFF();
				STATUS_LED_GREEN_OFF();

				// USB mode default config

				while(usb_activated){
					//USB LOGIC HERE

					if(led_delay < MICROS64_USB()){
						STATUS_LED_GREEN_TOGGLE();
						led_delay = MICROS64_USB() + 500 * 1000;
					}

					//USB LOGIC HERE

					// cheks if CLI is exited or USB is disconnected
					if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || exit_command_line_config){
						exit_command_line_config = false;
						usb_activated = false;
						FLIGHT_STATE_USB_DISABLE_OVERRIDE(FLIGHT_STATE_USB_OVERRIDE_KEY);
					}
				}

				STATUS_LED_GREEN_OFF();
				TIME_UTILS_CONTINUE_MICROS64();
				usb_config_mode_enabled = false;
			}

		}
		else{
			FLIGHT_STATE_USB_DISABLE_OVERRIDE(FLIGHT_STATE_USB_OVERRIDE_KEY);
		}
	}
}
