#include "LTR-329.h"

#include"stm32l4xx.h"// must add to have i2c type defines


static I2C_HandleTypeDef * LTR_329_I2C_HANDLE; //handle of which I2C it is ex: I2C2 or I2C1


uint8_t LTR_329_Init(I2C_HandleTypeDef * i2cInstance) {

	LTR_329_I2C_HANDLE = i2cInstance; // giving i2c handle instance

	HAL_StatusTypeDef i2cStatus ; // creating our error status varible

	// software reset of sensor
	i2cStatus = LTR_329_Reset();


	if (i2cStatus != HAL_OK) { // if the reset did not work then return error code 1

		return 1;

	}

	// Check device ID

	uint8_t deviceID = 0x00; // creating empty varible for the Device ID we are about to read

	i2cStatus = LTR_329_RegRead(LTR_329_PART_ID, &deviceID); // read device ID and put the read byte into the deviceID varible

	if( (i2cStatus != HAL_OK) || ((deviceID & 0xF0) != LTR_329_ID)){ // make sure it read the correct value by chekcing if the first 4 bits are the same as expected and was able to read

	return 2;// if something went wrong here return with error code 2

	}

	// write to the standby bit and change it from standby to active
	i2cStatus = LTR_329_RegWrite(LTR_329_ALS_CONTR, 0x01);

	if (i2cStatus != HAL_OK) {

		return 1;//if unable send back error code 1

	}
	return 0; // if all is well send back 0 indicating nothing went wrong
}

uint8_t LTR_329_Reset(){ // reset funtion

	HAL_StatusTypeDef i2cStatus;

	i2cStatus = LTR_329_RegWrite(LTR_329_ALS_CONTR, 0x02); // write to the software reset bit in the register

	HAL_Delay(25);

	if (i2cStatus != HAL_OK) {

		return 1;

	}
	return 0;
}


uint8_t LTR_329_Read_Data(LTR_329 *SunSensor){ // feed in adrdess to struct of sun sensor info

	HAL_StatusTypeDef i2cStatus; // create i2c status varible

		uint8_t Data0  = 0x00;// last half of 16 bit value from chanel 1
		uint8_t Data1 = 0x00;// first half of 16 bit value from chanel 1

		i2cStatus = LTR_329_RegRead(LTR_329_ALS_DATA_CH1_0, &Data0); // put data into Data0

		i2cStatus = LTR_329_RegRead(LTR_329_ALS_DATA_CH1_1, &Data1); // put data into Data1



		uint8_t Data2  = 0x00;// last half of 16 bit data from chanel 0
		uint8_t Data3 = 0x00;// first half of 16 bit data from chanel 0

		i2cStatus = LTR_329_RegRead(LTR_329_ALS_DATA_CH0_0, &Data2);// put data into Data2

		i2cStatus = LTR_329_RegRead(LTR_329_ALS_DATA_CH0_1, &Data3);// put data into Data3

		if (i2cStatus != HAL_OK) {

			return 1; // making sure the last register worked assuming the rest worked

		}

		SunSensor->CH1_Data = (Data1 << 8) | Data0 ;// Combining lower and upper bytes to give 16-bit Ch1 data
		SunSensor->CH2_Data = (Data3 << 8) | Data2 ;// Combining lower and upper bytes to give 16-bit Ch0 data

		return 0;// if all went well return 0 meaning all went well
}




HAL_StatusTypeDef LTR_329_RegWrite(uint8_t regAddr, uint8_t regData){
	// funtion for HAL i2c writing you can look this one up but we feeding which i2c we using, then sunsensor i2c adress, which register we want to mess with, size of data, and data, then how many byes we messing with (1) then max delay for it to try it
	return HAL_I2C_Mem_Write(LTR_329_I2C_HANDLE, LTR_329_I2C_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, &regData, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef LTR_329_RegRead(uint8_t regAddr, uint8_t *regData){
	// funtion for HAL i2c writing you can look this one up but we feeding which i2c we using, then sunsensor i2c adress, which register we want to mess with, size of data, and data, then how many byes we messing with (1) then max delay for it to try it
	return HAL_I2C_Mem_Read(LTR_329_I2C_HANDLE, LTR_329_I2C_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, regData, 1, HAL_MAX_DELAY);

}

