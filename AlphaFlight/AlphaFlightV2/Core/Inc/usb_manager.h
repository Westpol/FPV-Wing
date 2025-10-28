/*
 * usb_manager.h
 *
 *  Created on: Oct 28, 2025
 *      Author: benno
 */

#ifndef INC_USB_MANAGER_H_
#define INC_USB_MANAGER_H_

#include <stdint.h>
#include <stm32f7xx_hal.h>
#include "usbd_cdc_if.h"

extern uint8_t cdc_buffer[APP_RX_DATA_SIZE];

void USB_CHECK_FOR_CONNECTION();
void USB_INIT();
void USB_DECODE_MESSAGE(uint32_t length);

#endif /* INC_USB_MANAGER_H_ */
