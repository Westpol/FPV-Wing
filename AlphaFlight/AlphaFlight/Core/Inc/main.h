/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBAT_SENSE_Pin GPIO_PIN_0
#define VBAT_SENSE_GPIO_Port GPIOC
#define RGB_R_Pin GPIO_PIN_3
#define RGB_R_GPIO_Port GPIOC
#define SD_CD_Pin GPIO_PIN_4
#define SD_CD_GPIO_Port GPIOA
#define BARO_CS_Pin GPIO_PIN_4
#define BARO_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_0
#define GYRO_CS_GPIO_Port GPIOB
#define ACCEL_CS_Pin GPIO_PIN_1
#define ACCEL_CS_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_2
#define SERVO1_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_12
#define SERVO2_GPIO_Port GPIOB
#define SERVO3_Pin GPIO_PIN_14
#define SERVO3_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_15
#define SERVO4_GPIO_Port GPIOB
#define DSHOT_PIN_Pin GPIO_PIN_8
#define DSHOT_PIN_GPIO_Port GPIOA
#define SERVO7_Pin GPIO_PIN_13
#define SERVO7_GPIO_Port GPIOA
#define SEVO5_Pin GPIO_PIN_14
#define SEVO5_GPIO_Port GPIOA
#define SERVO6_Pin GPIO_PIN_15
#define SERVO6_GPIO_Port GPIOA
#define CS_EXT_3_Pin GPIO_PIN_3
#define CS_EXT_3_GPIO_Port GPIOB
#define CS_EXT_2_Pin GPIO_PIN_4
#define CS_EXT_2_GPIO_Port GPIOB
#define CS_EXT_1_Pin GPIO_PIN_5
#define CS_EXT_1_GPIO_Port GPIOB
#define RGB_B_Pin GPIO_PIN_8
#define RGB_B_GPIO_Port GPIOB
#define RGB_G_Pin GPIO_PIN_9
#define RGB_G_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
