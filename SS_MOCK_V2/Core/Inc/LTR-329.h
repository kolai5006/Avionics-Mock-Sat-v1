#pragma once
/* LTR_329 Sun Sensor Header file
 * June 6, 2025
 * Author: Adam Torrence
*/
#include"stm32l4xx.h" // must add to have i2c type defines

#define LTR_329_I2C_ADDR (0x29 << 1) //0010 1001 shifts to the left to gives you 0101 0010
                                    // 0x29 << 0x52
//Device ID Register Address
#define LTR_329_PART_ID 0x86

//Register address
#define LTR_329_ALS_CONTR 0x80
#define LTR_329_ALS_MEAS_RATE 0x85

#define LTR_329_MANUFAC_ID 0x87
#define LTR_329_ALS_DATA_CH1_0 0x88
#define LTR_329_ALS_DATA_CH1_1 0x89
#define LTR_329_ALS_DATA_CH0_0 0x8A
#define LTR_329_ALS_DATA_CH0_1 0x8B
#define LTR_329_ALS_STATUS 0x8C

//Device ID Value
#define LTR_329_ID 0xA0

//Struct that will contain all the data we care about
typedef struct {

	//settings
	uint8_t NewData_Status;
	uint8_t ALS_Gain;
	uint8_t ALS_Integration_Time;
	uint8_t ALS_mes_rate;

	// ADC Values
	uint16_t CH1_Data;
	uint16_t CH2_Data;


}LTR_329 ;


HAL_StatusTypeDef LTR_329_RegWrite(uint8_t regAddr, uint8_t regData); // read register function
HAL_StatusTypeDef LTR_329_RegRead(uint8_t regAddr, uint8_t *regData); // read register function
//functions
uint8_t LTR_329_Init(I2C_HandleTypeDef * i2cInstance); // run once on start up
uint8_t LTR_329_Reset(); // reset funtion

uint8_t LTR_329_Read_Data(LTR_329 *SunSensor); // function to read data
//Hi it's Dan and Ayleen and William


