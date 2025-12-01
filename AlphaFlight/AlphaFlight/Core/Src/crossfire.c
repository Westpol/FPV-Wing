#include "crossfire.h"

CRSF_Data_t CRSF_DATA = {0};  // Struct instance to hold CRSF channel data
static UART_HandleTypeDef *crsf_huart;  // Pointer to UART handler
static uint8_t crsf_rx_buffer[CRSF_BUFFER_SIZE];  // DMA buffer

void CRSF_Init(UART_HandleTypeDef *huart) {
    crsf_huart = huart;
    HAL_UART_Receive_DMA(crsf_huart, crsf_rx_buffer, CRSF_BUFFER_SIZE);  // Start UART DMA
}

// Get received byte count in DMA buffer
static uint16_t CRSF_GetRxCount() {
    return CRSF_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(crsf_huart->hdmarx);
}

// Decode CRSF channel data
static void CRSF_DecodeChannels(uint8_t *data) {
    CRSF_DATA.channels[0]  = (data[0] | data[1] << 8) & 0x07FF;
    CRSF_DATA.channels[1]  = (data[1] >> 3 | data[2] << 5) & 0x07FF;
    CRSF_DATA.channels[2]  = (data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF;
    CRSF_DATA.channels[3]  = (data[4] >> 1 | data[5] << 7) & 0x07FF;
    CRSF_DATA.channels[4]  = (data[5] >> 4 | data[6] << 4) & 0x07FF;
    CRSF_DATA.channels[5]  = (data[6] >> 7 | data[7] << 1 | data[8] << 9) & 0x07FF;
    CRSF_DATA.channels[6]  = (data[8] >> 2 | data[9] << 6) & 0x07FF;
    CRSF_DATA.channels[7]  = (data[9] >> 5 | data[10] << 3) & 0x07FF;
    CRSF_DATA.channels[8]  = (data[11] | data[12] << 8) & 0x07FF;
    CRSF_DATA.channels[9]  = (data[12] >> 3 | data[13] << 5) & 0x07FF;
    CRSF_DATA.channels[10] = (data[13] >> 6 | data[14] << 2 | data[15] << 10) & 0x07FF;
    CRSF_DATA.channels[11] = (data[15] >> 1 | data[16] << 7) & 0x07FF;
}

// Process CRSF data in main loop
void CRSF_Process() {
    uint16_t bytes_received = CRSF_GetRxCount();
    if (bytes_received < 3) return;  // Not enough data for a CRSF packet

    for (uint16_t i = 0; i < bytes_received - 2; i++) {
        if (crsf_rx_buffer[i] == 0xEA) {  // Start of an RC packet
            uint8_t length = crsf_rx_buffer[i + 1];
            if (length <= (bytes_received - i - 2)) {  // Check if full packet is available
                uint8_t type = crsf_rx_buffer[i + 2];
                if (type == 0x16) {  // RC Channel Data
                    CRSF_DecodeChannels(&crsf_rx_buffer[i + 3]);  // Extract channel values
                }
            }
        }
    }
}

// Get channel value (scaled to PWM range)
uint16_t CRSF_GetChannel(uint8_t channel) {
    if (channel >= NUM_CHANNELS) return 1500;  // Return midpoint if out of range
    return 988 + ((CRSF_DATA.channels[channel] * 988) / 2047);  // Scale to 1000-2000µs
}

// Optional: Restart DMA when transmission is complete
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == crsf_huart) {
        CRSF_Process();
        HAL_UART_Receive_DMA(crsf_huart, crsf_rx_buffer, CRSF_BUFFER_SIZE);  // Restart DMA
    }
}
