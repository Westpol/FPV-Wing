#include "usb_uart_passthrough.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

// Declare the UART handle for GPS (replace with your actual UART handle from CubeMX)
extern UART_HandleTypeDef huart1;  // Make sure to replace this with your actual UART handle

// Buffers for receiving and transmitting data
uint8_t usb_buffer[64];  // Buffer for USB data
uint8_t gps_buffer[128]; // Buffer for GPS data

// Function to transmit data from STM32 to the PC via USB CDC
uint8_t CDC_Transmit_FS(uint8_t* buf, uint16_t len) {
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buf, len);  // Set data buffer for transmission
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);  // Send data over USB
    return 0;
}

// Function to initialize USB UART passthrough
void USB_UART_Passthrough_Init(void) {
    // Start UART reception interrupt for GPS data
    HAL_UART_Receive_IT(&huart1, gps_buffer, sizeof(gps_buffer));  // Start receiving UART data

    // Setup USB CDC communication by setting the Tx buffer
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usb_buffer, 0);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);  // Begin USB transmission (initialization)

    // Optionally send an initialization message to PC
    // CDC_Transmit_FS((uint8_t*)"GPS Passthrough Initialized", 26);
}

// Callback for receiving data from USB CDC (PC to STM32)
void CDC_Receive_FS(uint8_t *buf, uint32_t *len) {
    // Copy the received data from USB buffer to the local USB buffer
    memcpy(usb_buffer, buf, *len);

    // Forward the received USB data to GPS via UART
    HAL_UART_Transmit(&huart1, usb_buffer, *len, 1000);
}

// UART callback for handling received GPS data (from GPS to STM32)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {  // Ensure to check the correct UART instance (e.g., USART1)
        // Process the received GPS data (you can add parsing or processing here if needed)

        // Forward received GPS data to the PC via USB CDC
        CDC_Transmit_FS(gps_buffer, sizeof(gps_buffer));

        // Restart UART reception to receive more data
        HAL_UART_Receive_IT(&huart1, gps_buffer, sizeof(gps_buffer));  // Re-enable UART reception interrupt
    }
}

// Function to start the passthrough process (call this once at startup or in the main loop)
void USB_UART_Passthrough_Run(void) {
    // Initialize the USB UART passthrough (UART and USB CDC setup)
    USB_UART_Passthrough_Init();
}
