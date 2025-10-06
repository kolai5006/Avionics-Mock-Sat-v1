#pragma once
/*
 * motorencoder.h
 *
 *  Created on: Sep 5, 2025
 *      Author: nicho
 */
#include "stm32l4xx.h"
#include "stm32l4xx_hal_tim.h"



#define ENCODER_MAX_COUNT 50000

typedef struct{
	float velocity;
	int32_t deltacount;
	uint32_t counter;
	uint32_t previousCounter;

} motor_var_enc;

void Motor_Enc_Init(motor_var_enc* s);

void Motor_Enc_Start(TIM_HandleTypeDef* htim);

//calc is short for calculator just some slang
void Motor_Calc(TIM_HandleTypeDef* htim, motor_var_enc* s);

void Motor_Reset(TIM_HandleTypeDef* htim, motor_var_enc* s, uint32_t counter_value);
