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
#include "stm32l4xx_hal.h"

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
#define WD_EN_Pin GPIO_PIN_3
#define WD_EN_GPIO_Port GPIOE
#define WWDG_Pin GPIO_PIN_6
#define WWDG_GPIO_Port GPIOE
#define MY_B_Pin GPIO_PIN_6
#define MY_B_GPIO_Port GPIOF
#define MY_A_Pin GPIO_PIN_7
#define MY_A_GPIO_Port GPIOF
#define MZ_B_Pin GPIO_PIN_6
#define MZ_B_GPIO_Port GPIOA
#define MZ_A_Pin GPIO_PIN_7
#define MZ_A_GPIO_Port GPIOA
#define IMON_3V3_Pin GPIO_PIN_5
#define IMON_3V3_GPIO_Port GPIOC
#define IMON_5V_Pin GPIO_PIN_0
#define IMON_5V_GPIO_Port GPIOB
#define VBAT_MON_Pin GPIO_PIN_1
#define VBAT_MON_GPIO_Port GPIOB
#define MCU_LED_Pin GPIO_PIN_0
#define MCU_LED_GPIO_Port GPIOG
#define BLE_STATE_Pin GPIO_PIN_1
#define BLE_STATE_GPIO_Port GPIOG
#define MZ_DIR_Pin GPIO_PIN_8
#define MZ_DIR_GPIO_Port GPIOE
#define MZ_PWM_Pin GPIO_PIN_9
#define MZ_PWM_GPIO_Port GPIOE
#define MY_DIR_Pin GPIO_PIN_10
#define MY_DIR_GPIO_Port GPIOE
#define MY_PWM_Pin GPIO_PIN_11
#define MY_PWM_GPIO_Port GPIOE
#define QSPI_CS2_Pin GPIO_PIN_12
#define QSPI_CS2_GPIO_Port GPIOB
#define MX_PWM_Pin GPIO_PIN_14
#define MX_PWM_GPIO_Port GPIOB
#define MX_DIR_Pin GPIO_PIN_15
#define MX_DIR_GPIO_Port GPIOB
#define MX_B_Pin GPIO_PIN_12
#define MX_B_GPIO_Port GPIOD
#define MX_A_Pin GPIO_PIN_13
#define MX_A_GPIO_Port GPIOD
#define FRAM_CS3_Pin GPIO_PIN_14
#define FRAM_CS3_GPIO_Port GPIOD
#define FRAM_CS2_Pin GPIO_PIN_15
#define FRAM_CS2_GPIO_Port GPIOD
#define FRAM_CS1_Pin GPIO_PIN_5
#define FRAM_CS1_GPIO_Port GPIOG
#define GPS_PPS_Pin GPIO_PIN_15
#define GPS_PPS_GPIO_Port GPIOA
#define GPS_FIX_Pin GPIO_PIN_12
#define GPS_FIX_GPIO_Port GPIOC
#define GPS_EN_Pin GPIO_PIN_0
#define GPS_EN_GPIO_Port GPIOD
#define USER_3_Pin GPIO_PIN_4
#define USER_3_GPIO_Port GPIOD
#define USER_2_Pin GPIO_PIN_5
#define USER_2_GPIO_Port GPIOD
#define USER_1_Pin GPIO_PIN_6
#define USER_1_GPIO_Port GPIOD
#define USER_0_Pin GPIO_PIN_7
#define USER_0_GPIO_Port GPIOD
#define I2C_EN_Pin GPIO_PIN_5
#define I2C_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
