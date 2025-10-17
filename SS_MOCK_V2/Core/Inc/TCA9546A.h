/**
 ******************************************************************************
 * @file    tca9546A.h
 * @brief   TCA9546A I2C Switch Driver for STM32L496
 * @author  Ayleen Perez
 * @date    October 17, 2025
 ******************************************************************************
 * @attention
 * - Supports Standard Mode (100kHz) and Fast Mode (400kHz)
 * - Uses STM32L4 HAL driver
 ******************************************************************************
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "stm32l4xx_hal.h"

/* Error Codes */
typedef enum
{
    TCA9546A_SUCCESS = 0U,      // everything works
    TCA9546A_NOT_DETECTED = 1U, // cant find device
    TCA9546A_ERROR = 2U         // generic error
} TCA9546A_StatusTypeDef;

/* Channel Definitions */
typedef enum
{
    TCA9546A_CHANNEL_0 = 0x01U, // binary 00000001
    TCA9546A_CHANNEL_1 = 0x02U, // binary 00000010
    TCA9546A_CHANNEL_2 = 0x04U, // binary 00000100
    TCA9546A_CHANNEL_3 = 0x08U, // binary 00001000
    TCA9546A_NO_CHANNEL = 0x00U
} TCA9546A_CHANNEL;

/* Optional Hardware Reset Support */
// #define TCA9546A_HARDWARE_RESET

/* Function Prototypes */

/**
 * @brief Mount I2C handle to driver
 * @param i2c Pointer to I2C handle (I2C1, I2C2, I2C3, or I2C4)
 */
void TCA9546A_I2C_Mount(I2C_HandleTypeDef *i2c);

#ifdef TCA9546A_HARDWARE_RESET
/**
 * @brief Mount hardware reset pin
 * @param GPIO_Port GPIO port
 * @param GPIO_Pin GPIO pin
 */
void TCA9546A_HW_Reset_Mount(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin);
#endif

/**
 * @brief Initialize TCA9546A device
 * @param a2 State of A2 pin (0 or 1)
 * @param a1 State of A1 pin (0 or 1)
 * @param a0 State of A0 pin (0 or 1)
 * @return TCA9546A_StatusTypeDef status
 */
TCA9546A_StatusTypeDef TCA9546A_Init(uint8_t a2, uint8_t a1, uint8_t a0);

/**
 * @brief Select one or multiple channels
 * @param channels Channel(s) to enable (bitwise OR)
 * @return TCA9546A_StatusTypeDef status
 */
TCA9546A_StatusTypeDef TCA9546A_Select_Channels(uint8_t channels);

/**
 * @brief Enable single channel (disables others)
 * @param channel Channel to enable
 * @return TCA9546A_StatusTypeDef status
 */
TCA9546A_StatusTypeDef TCA9546A_Enable_Channel(TCA9546A_CHANNEL channel);

/**
 * @brief Disable all channels
 * @return TCA9546A_StatusTypeDef status
 */
TCA9546A_StatusTypeDef TCA9546A_Disable_All_Channels(void);

/**
 * @brief Add channel(s) to current selection
 * @param channels Channel(s) to add
 * @return TCA9546A_StatusTypeDef status
 */
TCA9546A_StatusTypeDef TCA9546A_Add_Channels(uint8_t channels);

/**
 * @brief Remove channel(s) from current selection
 * @param channels Channel(s) to remove
 * @return TCA9546A_StatusTypeDef status
 */
TCA9546A_StatusTypeDef TCA9546A_Remove_Channels(uint8_t channels);

/**
 * @brief Read current channel selection
 * @param channels Pointer to store channel data
 * @return TCA9546A_StatusTypeDef status
 */
TCA9546A_StatusTypeDef TCA9546A_Get_Channels(uint8_t *channels);

/**
 * @brief Check if device is present
 * @return true if present, false otherwise
 */
bool TCA9546A_Is_Present(void);
