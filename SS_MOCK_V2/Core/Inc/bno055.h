
#ifndef BNO055_H_
#define BNO055_H_

#include <stdint.h>
#include <stdlib.h>

#include "stm32l4xx_hal.h"

typedef enum{
	BNO055_SUCCESS        = 0U,
	BNO055_NOT_DETECTED   = 1U,
} BNO055_ERROR;

typedef enum{
	CONFIG,
	ACC_ONLY,
	MAG_ONLY,
	GYRO_ONLY,
	ACC_MAG,
	ACC_GYRO,
	MAG_GYRO,
	AMG,
	IMU,
	COMPASS,
	M4G,
	NDOF_FMC_OFF,
	NDOF
} BNO055_OPERATION_MODE;

typedef enum{
	METER_PER_SEC_SQUARED = 0x00U,
	MILLI_G				  = 0x01U
} BNO055_ACCEL_GRAV_UNIT;

typedef enum{
	Dps	= 0x00U << 1,
	Rps = 0x01U << 1
} BNO055_ANGULAR_RATE_UNIT;

typedef enum{
	DEGREES = 0x00U << 2,
	RADIANS = 0x01U << 2
} BNO055_EULER_UNIT;

typedef enum{
	CELCIUS 	= 0x00U << 4,
	FAHRENHEIT  = 0x01U << 4
} BNO055_TEMP_UNIT;

typedef enum{
	BNO055_X_AXIS = 0x00U,
	BNO055_Y_AXIS = 0x01U,
	BNO055_Z_AXIS = 0x02U
} BNO055_AXIS;

typedef struct{
	float x;
	float y;
	float z;
} BNO055_Axis_Vec_t;

typedef struct{
	float w;
	float x;
	float y;
	float z;
} BNO055_Quad_Vec_t;

typedef struct{
	BNO055_AXIS x;
	BNO055_AXIS y;
	BNO055_AXIS z;
} BNO055_AXIS_CONFIG_t;

//#define BNO055_HARDWARE_RESET

void BNO055_I2C_Mount(I2C_HandleTypeDef* i2c);

#ifdef BNO055_HARDWARE_RESET
	void BNO055_HW_Reset_Mount(uint32_t GPIO_Port, uint16_t GPIO_Pin);
#endif

BNO055_ERROR BNO055_Init(void);
BNO055_ERROR BNO055_Set_Unit(BNO055_ACCEL_GRAV_UNIT accUnit, BNO055_ANGULAR_RATE_UNIT angRateUnit, BNO055_EULER_UNIT eulerUnit, BNO055_TEMP_UNIT tempUnit);
BNO055_ERROR BNO055_Set_OP_Mode(BNO055_OPERATION_MODE op);
BNO055_ERROR BNO055_Set_Axis(const BNO055_AXIS_CONFIG_t* axis_struct);
BNO055_ERROR BNO055_Calibrate(void);
BNO055_ERROR BNO055_Get_Accel(BNO055_Axis_Vec_t* vec);
BNO055_ERROR BNO055_Get_Gyro(BNO055_Axis_Vec_t* vec);
BNO055_ERROR BNO055_Get_Mag(BNO055_Axis_Vec_t* vec);
BNO055_ERROR BNO055_Get_Linear_Accel(BNO055_Axis_Vec_t* vec);
BNO055_ERROR BNO055_Get_Gravity_Vec(BNO055_Axis_Vec_t* vec);
BNO055_ERROR BNO055_Get_Temp(uint8_t temp);
BNO055_ERROR BNO055_Get_Euler_Vec(BNO055_Axis_Vec_t* vec);
BNO055_ERROR BNO055_Get_Quaternion(BNO055_Quad_Vec_t* vec);

#endif /* INC_BNO055_H_ */
