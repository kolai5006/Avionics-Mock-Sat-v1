/*
 * mcp9808.c
 *
 *  Created on: Aug 29, 2025
 *      Author: nicho
 */
#include "mcp9808.h"
#include "stm32l4xx.h"


static I2C_HandleTypeDef * MCP_9808_I2C_HANDLE;

uint8_t MCP_9808_Init(I2C_HandleTypeDef * i2cInstance){

	MCP_9808_I2C_HANDLE = i2cInstance;

	HAL_StatusTypeDef i2cStatus;

	//check device ID
	//should return 0x0054
	uint16_t deviceID = 0x00;

	i2cStatus =  MCP_9808_RegRead(MCP_9808_REG_MANUF_ID, &deviceID);

	if ((i2cStatus != HAL_OK) || ((deviceID)!= 0x0054)){

		return 2;
	}

	return 0;

}


//reads temperature
uint16_t MCP_9808_Read(MCP_9808 * TempSensor){

	HAL_StatusTypeDef i2cStatus;

	//Writes 16bit data
	uint8_t UpperTemp = 0x00;
	uint8_t LowerTemp = 0x00;

	i2cStatus =  MCP_9808_RegRead(MCP_9808_REG_UPPER_TEMP,&UpperTemp);
	i2cStatus =  MCP_9808_RegRead(MCP_9808_REG_LOWER_TEMP, &LowerTemp);

	if (i2cStatus != HAL_OK){

		return 1;
	}

	TempSensor->UPPER_TEMP = (UpperTemp);
	TempSensor->LOWER_TEMP = (LowerTemp);

	//returns 0 if no error
	return 0;

}
//write
HAL_StatusTypeDef MCP_9808_RegWrite(uint16_t regAddr, uint16_t regData){

	return HAL_I2C_Mem_Write(MCP_9808_I2C_HANDLE, MCP_9808_I2CADDR_DEFAULT, regAddr, I2C_MEMADD_SIZE_8BIT, &regData, 2, HAL_MAX_DELAY);
}
//read //fix the I2c HANDLE

HAL_StatusTypeDef MCP_9808_RegRead(uint16_t regAddr, uint16_t* regData){

	return HAL_I2C_Mem_Read(MCP_9808_I2C_HANDLE, MCP_9808_I2CADDR_DEFAULT, regAddr, I2C_MEMADD_SIZE_8BIT, regData, 2, HAL_MAX_DELAY);

}
