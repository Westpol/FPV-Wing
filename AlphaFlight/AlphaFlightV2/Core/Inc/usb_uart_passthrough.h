#ifndef USB_UART_PASSTHROUGH_H
#define USB_UART_PASSTHROUGH_H

#include "stm32f7xx_hal.h"

// Function to transmit data from STM32 to PC via USB CDC
uint8_t CDC_Transmit_FS(uint8_t* buf, uint16_t len);

// Function to initialize USB UART passthrough
void USB_UART_Passthrough_Init(void);

// Callback for receiving data from USB CDC (PC to STM32)
void CDC_Receive_FS(uint8_t *buf, uint32_t *len);

// UART interrupt callback (for receiving GPS data from UART)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// Function to start the passthrough process (call once at startup or in the main loop)
void USB_UART_Passthrough_Run(void);

#endif /* USB_UART_PASSTHROUGH_H */
