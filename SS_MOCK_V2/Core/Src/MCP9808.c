/*
 * mcp9808.c
 *
 *  Created on: Oct 8, 2025
 *      Author: nicho
 */

#include "mcp9808.h"
#include "stm32l4xx.h"


static I2C_HandleTypeDef * MCP_9808_I2C_HANDLE;


uint8_t MCP_9808_Init(I2C_HandleTypeDef * i2cInstance){

	MCP_9808_I2C_HANDLE = i2cInstance;

	HAL_StatusTypeDef i2cStatus;

	uint16_t deviceID = 0x0000;

	i2cStatus = MCP_9808_RegRead16(MCP_9808_REG_MANUF_ID, &deviceID);  // use 16-bit read

	if ((i2cStatus != HAL_OK) || (deviceID != 0x0054)){

		return 2;
	}

	return 0;

}


//calc(short for calculator) temperature in celsius, then computes in fahrenheit
float MCP_9808_CalcTemp(MCP_9808 * TempSensor){

	uint16_t rawTemp = TempSensor->rawTemp;

	// clear alert flags (upper 3 bits)
	rawTemp = rawTemp & 0x1FFF;

	float temperature = 0.0;
	float farenheit = 0.0;

	// check if temperature is negative (bit 12 is set)
	if (rawTemp & 0x1000){

		// negative temperature
		rawTemp = rawTemp & 0x0FFF;  // clear sign bit
		temperature = 256.0 - (rawTemp * 0.0625);

	} else {

		// positive temperature
		temperature = rawTemp * 0.0625;
	}

	TempSensor->temperatureC = temperature;

	farenheit = (temperature * 1.8) + 32;
	TempSensor->temperatureF = farenheit;


	return temperature;
}

//read temp
uint8_t MCP_9808_ReadTemp(MCP_9808 * TempSensor){

	HAL_StatusTypeDef i2cStatus;

	uint16_t ambientTemp = 0x0000;

	i2cStatus = MCP_9808_RegRead16(MCP_9808_REG_AMBIENT_TEMP, &ambientTemp);

	if (i2cStatus != HAL_OK){

		return 1;
	}

	TempSensor->rawTemp = ambientTemp;

	// automatically calc(short for calculator) temperature
	MCP_9808_CalcTemp(TempSensor);

	return 0;

}

//write 16-bit
HAL_StatusTypeDef MCP_9808_RegWrite16(uint8_t regAddr, uint16_t regData){

	uint8_t buffer[2];

	buffer[0] = (regData >> 8) & 0xFF;  // upper byte
	buffer[1] = regData & 0xFF;          // lower byte

	return HAL_I2C_Mem_Write(MCP_9808_I2C_HANDLE, MCP_9808_I2CADDR_DEFAULT, regAddr, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY);
}

//read 16-bit
HAL_StatusTypeDef MCP_9808_RegRead16(uint8_t regAddr, uint16_t* regData){

	uint8_t buffer[2];

	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(MCP_9808_I2C_HANDLE, MCP_9808_I2CADDR_DEFAULT, regAddr, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY);

	if (status == HAL_OK){

		*regData = (buffer[0] << 8) | buffer[1];  // combine bytes
	}

	return status;
}

//write 8-bit
HAL_StatusTypeDef MCP_9808_RegWrite(uint8_t regAddr, uint8_t regData){

	return HAL_I2C_Mem_Write(MCP_9808_I2C_HANDLE, MCP_9808_I2CADDR_DEFAULT, regAddr, I2C_MEMADD_SIZE_8BIT, &regData, 1, HAL_MAX_DELAY);
}

//read 8-bit
HAL_StatusTypeDef MCP_9808_RegRead(uint8_t regAddr, uint8_t* regData){

	return HAL_I2C_Mem_Read(MCP_9808_I2C_HANDLE, MCP_9808_I2CADDR_DEFAULT, regAddr, I2C_MEMADD_SIZE_8BIT, regData, 1, HAL_MAX_DELAY);

}
