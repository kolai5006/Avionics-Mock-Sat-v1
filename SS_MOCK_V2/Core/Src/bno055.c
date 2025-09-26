/*
 * BNO055.c
 *
 *  Created on: Sep 4, 2023
 *      Author: baske
 */

#include "bno055.h"
#include "bno055_regs.h"

I2C_HandleTypeDef* BNO055_I2C;

typedef struct{

	BNO055_OPERATION_MODE currentMode;
	uint8_t currentPage;

} BNO055_STATE;

BNO055_STATE state;

#ifdef BNO055_HARDWARE_RESET

uint32_t Reset_Port = 0;
uint16_t Reset_Pin = 0;

#endif

/*
 * Local Read Function to BNO055
 */
static inline HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t* buf, uint8_t len){

	HAL_StatusTypeDef error;

	error = HAL_I2C_Master_Transmit(BNO055_I2C, BNO055_I2C_ADDR<<1, &reg, 1, BNO055_WRITE_TIMEOUT);
	error |= HAL_I2C_Master_Receive(BNO055_I2C, BNO055_I2C_ADDR<<1, buf, len, BNO055_READ_TIMEOUT);

	return error;

}

/*
 * Local Write Function to BNO055
 */
static inline HAL_StatusTypeDef BNO055_Write(uint8_t reg, uint8_t data){

	HAL_StatusTypeDef error;
	uint8_t transmitBuf[2] = {reg, data};

	error = HAL_I2C_Master_Transmit(BNO055_I2C, BNO055_I2C_ADDR<<1, transmitBuf, sizeof(transmitBuf), BNO055_WRITE_TIMEOUT);

	return error;

}

/*
 * Simple Error Check Function
 */
static inline void BNO055_ERROR_HANDLE(HAL_StatusTypeDef error){
	if(error != HAL_OK){
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		while(1);
	}
}

/**
 * Check Current Mode and Page 
 * Set Current Mode and Page to new mode and poge
 * Updates state struct instance
 */
static inline void BNO055_STATUS_CHECK_AND_SET(BNO055_STATE* currState, BNO055_OPERATION_MODE newMode, uint8_t newPage){

	HAL_StatusTypeDef error;

	/* Check and Set Current Mode to new Mode */
	if(currState->currentMode != newMode){
		error = BNO055_Write(BNO055_OPR_MODE, newMode);
		BNO055_ERROR_HANDLE(error);
		currState->currentMode = newMode;
		HAL_Delay(20);
	}

	/* Check and Set current page to new page */
	if(currState->currentPage != newPage){
		error = BNO055_Write(BNO055_PAGE_ID, newPage);
		BNO055_ERROR_HANDLE(error);
		currState->currentPage = newPage;
		HAL_Delay(20);
	}
}

/*
 * Allow User to Mount I2C Handle
 */
void BNO055_I2C_Mount(I2C_HandleTypeDef* i2c){
	BNO055_I2C = i2c;
}

#ifdef BNO055_HARDWARE_RESET

void BNO055_HW_Reset_Mount(uint32_t GPIO_Port, uint16_t GPIO_Pin){
	Reset_Port = GPIO_Port;
	Reset_Pin = GPIO_Pin;
}

#endif

BNO055_ERROR BNO055_Init(void){


	HAL_Delay(700);
	HAL_StatusTypeDef error;
	uint8_t ID_Check = 0;

	/* Read Chip ID Register to see if BNO055 is detected*/
	error = BNO055_Read(BNO055_CHIP_ID, &ID_Check, 1);
	BNO055_ERROR_HANDLE(error);
	if(ID_Check != BNO055_EXPECTED_ID)
		return BNO055_NOT_DETECTED;

	/* Software Reset Module */
	error = BNO055_Write(BNO055_SYS_TRIGGER, BNO055_SYS_RESET);
	BNO055_ERROR_HANDLE(error);
	HAL_Delay(700);													//Necessary Delay when Soft Reset

	/* Hardware Reset Module */
	#ifdef BNO055_HARDWARE_RESET
		HAL_GPIO_WritePin(Reset_Port, Reset_Pin, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(Reset_Port, Reset_Pin, GPIO_PIN_SET);
	#endif

	/* Set Register Page to 0 and Clear System Trigger*/
	error = BNO055_Write(BNO055_PAGE_ID, BNO055_PAGE_0);
	BNO055_ERROR_HANDLE(error);
	state.currentPage = BNO055_PAGE_0;
	HAL_Delay(20);

	/* Set BNO055 Operation Mode to CONFIG MODE*/
	error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_CONFIG);
	BNO055_ERROR_HANDLE(error);
	state.currentMode = CONFIG;

	/* Necessary Delay to Set Operation Mode */
	HAL_Delay(10);

	error = BNO055_Write(BNO055_SYS_TRIGGER, 0x01);
	HAL_Delay(100);

	error = BNO055_Write(BNO055_PAGE_ID, BNO055_PAGE_0);
		BNO055_ERROR_HANDLE(error);
		state.currentPage = BNO055_PAGE_0;
		HAL_Delay(20);

	uint8_t ST_Result = 0;
	error = BNO055_Read(BNO055_ST_RESULT, &ST_Result , 1);


	return BNO055_SUCCESS;

}

BNO055_ERROR BNO055_Set_Unit(BNO055_ACCEL_GRAV_UNIT accUnit, BNO055_ANGULAR_RATE_UNIT angRateUnit, BNO055_EULER_UNIT eulerUnit, BNO055_TEMP_UNIT tempUnit){

	HAL_StatusTypeDef error;

	/* Makes sure BNO055 is in CONFIG mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, CONFIG, BNO055_PAGE_0);

	/* Set Unit */
	uint8_t temp = accUnit | angRateUnit | eulerUnit | tempUnit;
	error = BNO055_Write(BNO055_UNIT_SEL, temp);
	BNO055_ERROR_HANDLE(error);

	return BNO055_SUCCESS;
}

BNO055_ERROR BNO055_Set_OP_Mode(BNO055_OPERATION_MODE op){

	HAL_StatusTypeDef error;

	/* Make sure BNO055 is on Page 0 */
	if(state.currentPage != BNO055_PAGE_0){
		error = BNO055_Write(BNO055_PAGE_ID, BNO055_PAGE_0);
		BNO055_ERROR_HANDLE(error);
		state.currentPage = BNO055_PAGE_0;
		HAL_Delay(20);
	}

	switch(op){
		case ACC_ONLY:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_ACCONLY);
			break;

		case MAG_ONLY:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_MAGONLY);
			break;

		case GYRO_ONLY:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_GRYOONLY);
			break;

		case ACC_MAG:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_ACCMAG);
			break;

		case ACC_GYRO:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_ACCGYRO);
			break;

		case MAG_GYRO:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_MAGGRYO);
			break;

		case IMU:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_IMU);
			break;

		case COMPASS:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_COMPASS);
			break;

		case M4G:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_M4G);
			break;

		case NDOF_FMC_OFF:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_NDOF_FMC);
			break;

		case NDOF:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_NDOF);
			break;

		default:
			error = BNO055_Write(BNO055_OPR_MODE, OPR_MODE_CONFIG);
			break;
	}

	BNO055_ERROR_HANDLE(error);
	state.currentMode = op;
	HAL_Delay(10);
	return BNO055_SUCCESS;
}

BNO055_ERROR BNO055_Set_Axis(const BNO055_AXIS_CONFIG_t* axesConfig){
	
	HAL_StatusTypeDef error; 

	/* Makes sure BNO055 is in CONFIG mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, CONFIG, BNO055_PAGE_0);

	/* Configure Axis based on Axis Config Struct */
	uint8_t temp = (axesConfig->x << BNO055_X_AXIS_OFFSET) |
				   (axesConfig->y << BNO055_Y_AXIS_OFFSET) |
				   (axesConfig->z << BNO055_Z_AXIS_OFFSET);

	error = BNO055_Write(BNO055_AXIS_MAP_CONFIG, temp);
	BNO055_ERROR_HANDLE(error);
	HAL_Delay(10);

	return BNO055_SUCCESS;
}

BNO055_ERROR BNO055_Calibrate(void){

	return BNO055_SUCCESS;

}

BNO055_ERROR BNO055_Get_Accel(BNO055_Axis_Vec_t* vec){

	HAL_StatusTypeDef error;
	uint8_t tempBuf[6];

	/* Makes sure BNO055 is in NDOF mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, NDOF, BNO055_PAGE_0);

	/* Grab Accel LSB and MSB data of all axes */
	error = BNO055_Read(BNO055_ACC_DATA_X_LSB, tempBuf, 6);
	BNO055_ERROR_HANDLE(error);

	/* Parse Data into Struct */
	vec->x = (int16_t)((tempBuf[1]<<8) | tempBuf[0]) / BN0055_ACCEL_MS2_SCALE;
	vec->y = (int16_t)((tempBuf[3]<<8) | tempBuf[2]) / BN0055_ACCEL_MS2_SCALE;
	vec->z = (int16_t)((tempBuf[5]<<8) | tempBuf[4]) / BN0055_ACCEL_MS2_SCALE;

	return BNO055_SUCCESS;

}

BNO055_ERROR BNO055_Get_Gyro(BNO055_Axis_Vec_t* vec){

	HAL_StatusTypeDef error;
	uint8_t tempBuf[6];

	/* Makes sure BNO055 is in NDOF mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, NDOF, BNO055_PAGE_0);

	/* Grab Gyro LSB and MSB data of all axes */
	error = BNO055_Read(BNO055_GYR_DATA_X_LSB, tempBuf, 6);
	BNO055_ERROR_HANDLE(error);

	/* Parse Data into Vector Struct */
	vec->x = (int16_t)((tempBuf[1]<<8) | tempBuf[0]) / BNO055_GYRO_DPS_SCALE;
	vec->y = (int16_t)((tempBuf[3]<<8) | tempBuf[2]) / BNO055_GYRO_DPS_SCALE;
	vec->z = (int16_t)((tempBuf[5]<<8) | tempBuf[4]) / BNO055_GYRO_DPS_SCALE;

	return BNO055_SUCCESS;

}

BNO055_ERROR BNO055_Get_Mag(BNO055_Axis_Vec_t* vec){

	HAL_StatusTypeDef error;
	uint8_t tempBuf[6];

	/* Makes sure BNO055 is in NDOF mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, NDOF, BNO055_PAGE_0);

	/* Grab Mag LSB and MSB data of all axes */
	error = BNO055_Read(BNO055_MAG_DATA_X_LSB, tempBuf, 6);
	BNO055_ERROR_HANDLE(error);

	/* Parse Data into Vector Struct */
	vec->x = (int16_t)((tempBuf[1]<<8) | tempBuf[0]) / BNO055_MAG_SCALE;
	vec->y = (int16_t)((tempBuf[3]<<8) | tempBuf[2]) / BNO055_MAG_SCALE;
	vec->z = (int16_t)((tempBuf[5]<<8) | tempBuf[4]) / BNO055_MAG_SCALE;

	return BNO055_SUCCESS;

}

BNO055_ERROR BNO055_Get_Linear_Accel(BNO055_Axis_Vec_t* vec){

	HAL_StatusTypeDef error;
	uint8_t tempBuf[6];

	/* Makes sure BNO055 is in NDOF mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, NDOF, BNO055_PAGE_0);

	/* Grab Linear Acceleration LSB and MSB data of all axes */
	error = BNO055_Read(BNO055_LIA_DATA_X_LSB, tempBuf, 6);
	BNO055_ERROR_HANDLE(error);

	/* Parse Data into Vector Struct */
	vec->x = (int16_t)((tempBuf[1]<<8) | tempBuf[0]) / BNO055_LIN_ACCEL_MS2_SCALE;
	vec->y = (int16_t)((tempBuf[3]<<8) | tempBuf[2]) / BNO055_LIN_ACCEL_MS2_SCALE;
	vec->z = (int16_t)((tempBuf[5]<<8) | tempBuf[4]) / BNO055_LIN_ACCEL_MS2_SCALE;

	return BNO055_SUCCESS;

}

BNO055_ERROR BNO055_Get_Gravity_Vec(BNO055_Axis_Vec_t* vec){

	HAL_StatusTypeDef error;
	uint8_t tempBuf[6];

	/* Makes sure BNO055 is in NDOF mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, NDOF, BNO055_PAGE_0);

	/* Grab Gravity LSB and MSB data of all axes */
	error = BNO055_Read(BNO055_GRV_DATA_X_LSB, tempBuf, 6);
	BNO055_ERROR_HANDLE(error);

	/* Parse Data into Vector Struct */
	vec->x = (int16_t)((tempBuf[1]<<8) | tempBuf[0]) / BNO055_GRAV_MG2_SCALE;
	vec->y = (int16_t)((tempBuf[3]<<8) | tempBuf[2]) / BNO055_GRAV_MG2_SCALE;
	vec->z = (int16_t)((tempBuf[5]<<8) | tempBuf[4]) / BNO055_GRAV_MG2_SCALE;

	return BNO055_SUCCESS;

}

BNO055_ERROR BNO055_Get_Temp(uint8_t temp){

	HAL_StatusTypeDef error;

	/* Makes sure BNO055 is in NDOF mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, NDOF, BNO055_PAGE_0);

	/* Grab Temperature data */
	error = BNO055_Read(BNO055_TEMP, &temp, 1);
	BNO055_ERROR_HANDLE(error);

	return BNO055_SUCCESS;

}

BNO055_ERROR BNO055_Get_Euler_Vec(BNO055_Axis_Vec_t* vec){

	HAL_StatusTypeDef error;
	uint8_t tempBuf[6];

	/* Makes sure BNO055 is in NDOF mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, NDOF, BNO055_PAGE_0);

	/* Grab Euler Vector Data LSB and MSB */
	error = BNO055_Read(BNO055_EUL_HEADING_LSB, tempBuf, sizeof(tempBuf));
	BNO055_ERROR_HANDLE(error);

	/* Parse Data Into Euler Vector Struct */
	vec->x = (int16_t)((tempBuf[1]<<8) | tempBuf[0]) / BNO055_EULER_DEGREE_SCALE;
	vec->y = (int16_t)((tempBuf[3]<<8) | tempBuf[2]) / BNO055_EULER_DEGREE_SCALE;
	vec->z = (int16_t)((tempBuf[5]<<8) | tempBuf[4]) / BNO055_EULER_DEGREE_SCALE;

	return BNO055_SUCCESS;
}

BNO055_ERROR BNO055_Get_Quaternion(BNO055_Quad_Vec_t* vec){

	HAL_StatusTypeDef error;
	uint8_t tempBuf[8];

	/* Makes sure BNO055 is in NDOF mode and Page 0 */
	BNO055_STATUS_CHECK_AND_SET(&state, NDOF, BNO055_PAGE_0);

	/* Grab Quaternion Vector LSB and MSB data */
	error = BNO055_Read(BNO055_QUA_DATA_W_LSB, tempBuf, sizeof(tempBuf));
	BNO055_ERROR_HANDLE(error);

	/* Parse Data Into Quad Vector Struct */
	vec->w = (int16_t)((tempBuf[1]<<8) | tempBuf[0]) / BNO055_QUATERNION_SCALE;
	vec->x = (int16_t)((tempBuf[3]<<8) | tempBuf[2]) / BNO055_QUATERNION_SCALE;
	vec->y = (int16_t)((tempBuf[5]<<8) | tempBuf[4]) / BNO055_QUATERNION_SCALE;
	vec->z = (int16_t)((tempBuf[7]<<8) | tempBuf[6]) / BNO055_QUATERNION_SCALE;

	return BNO055_SUCCESS;
}
