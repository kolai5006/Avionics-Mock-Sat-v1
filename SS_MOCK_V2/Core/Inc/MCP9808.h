#pragma once
/*
 * mcp9808.h
 *
 *  Created on: July 22, 2025
 *      Author: nicho
 */

#include "stm32l4xx_hal.h"

//I2C Address
#define MCP_9808_I2CADDR_DEFAULT (0x18<<1)  ///< I2C address
#define MCP_9808_REG_CONFIG 0x01      ///< MCP9808 config register


#define MCP_9808_REG_CONFIG_SHUTDOWN 0x0100   ///< shutdown config
#define MCP_9808_REG_CONFIG_CRITLOCKED 0x0080 ///< critical trip lock
#define MCP_9808_REG_CONFIG_WINLOCKED 0x0040  ///< alarm window lock
#define MCP_9808_REG_CONFIG_INTCLR 0x0020     ///< interrupt clear
#define MCP_9808_REG_CONFIG_ALERTSTAT 0x0010  ///< alert output status
#define MCP_9808_REG_CONFIG_ALERTCTRL 0x0008  ///< alert output control
#define MCP_9808_REG_CONFIG_ALERTSEL 0x0004   ///< alert output select
#define MCP_9808_REG_CONFIG_ALERTPOL 0x0002   ///< alert output polarity
#define MCP_9808_REG_CONFIG_ALERTMODE 0x0001  ///< alert output mode


#define MCP_9808_REG_UPPER_TEMP 0x02   ///< upper alert boundary
#define MCP_9808_REG_LOWER_TEMP 0x03   ///< lower alert boundary

#define MCP_9808_REG_CRIT_TEMP 0x04    ///< critical temperature
#define MCP_9808_REG_AMBIENT_TEMP 0x05 ///< ambient temperature

#define MCP_9808_REG_MANUF_ID 0x06     ///< manufacture ID
#define MCP_9808_REG_DEVICE_ID 0x07    ///< device ID
#define MCP_9808_REG_RESOLUTION 0x08


typedef struct{

	//update later
	uint8_t NewData_Status;
	uint8_t ALS_Gain;
	uint8_t ALS_Integration_time;
	uint8_t ALS_mes_rate;

	uint16_t UPPER_TEMP;
	uint16_t LOWER_TEMP;

} MCP_9808;

//reset values are stored in here
//extern uint8_t MCP9808_REG_CONFIG_SETTINGS[8];

//functions
uint8_t MCP_9808_Init(I2C_HandleTypeDef * i2cInstance);
uint8_t MCP_9808_Reset();

uint16_t MCP_9808_Read(MCP_9808 * TempSensor);

uint16_t MCP_9808_Read_UPPER(I2C_HandleTypeDef * i2cInstance);
uint16_t MCP_9808_Read_LOWER(I2C_HandleTypeDef * i2cInstance);

HAL_StatusTypeDef MCP_9808_RegWrite(uint16_t regAddr, uint16_t regData);
HAL_StatusTypeDef MCP_9808_RegRead(uint16_t regAddr, uint16_t *regData);




