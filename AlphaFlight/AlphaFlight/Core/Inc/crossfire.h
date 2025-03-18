
#ifndef __CROSSFIRE_H
#define __CROSSFIRE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"

#define CRSF_RADIO_CONTROL_STARTBYTE 0xEE
#define CRSF_BUFFER_SIZE 64
#define NUM_CHANNELS 16

typedef struct {
    uint16_t channels[NUM_CHANNELS];
} CRSF_Data_t;

extern CRSF_Data_t crsf_data;

void CRSF_Init(UART_HandleTypeDef *huart);

void CRSF_Process(void);

uint16_t CRSF_GetChannel(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif
