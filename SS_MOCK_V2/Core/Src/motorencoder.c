/*
 * motorencoder.c
 *
 *  Created on: Sep 5, 2025
 *      Author: nicho
 */

#include "motorencoder.h"

//initializing variables
void Motor_Enc_Init(motor_var_enc* s){

	s-> velocity = 0;
	s-> deltacount = 0;
	s-> counter = 0;
	s-> previousCounter = 0;

}

//
void Motor_Enc_Start(TIM_HandleTypeDef* htim){

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

}

//short for calculator
//
void Motor_Calc(TIM_HandleTypeDef* htim, motor_var_enc* s){

	s->counter = __HAL_TIM_GET_COUNTER(htim);

	s->deltacount = (int32_t)s->counter - (int32_t)s->previousCounter;


	if (s->deltacount <= - ENCODER_MAX_COUNT/2){
		s->deltacount += (ENCODER_MAX_COUNT + 1);


	}else if (s->deltacount >= ENCODER_MAX_COUNT/2){
		s->deltacount -= (ENCODER_MAX_COUNT - 1);

	}

	s->velocity = (float)s->deltacount / 0.01f;

	s->previousCounter = s->counter;




    // Calculate difference in counts since the last interrupt
/*    deltacount = (int32_t)counter - (int32_t)previousCounter;

    if (deltacount <= -ENCODER_MAX_COUNT / 2){
        deltacount += ENCODER_MAX_COUNT + 1;
    }
    // If the difference is very positive, it means the counter wrapped backward
    else if (deltacount >= ENCODER_MAX_COUNT / 2){
        deltacount -= ENCODER_MAX_COUNT - 1;
    }

    velocity = (float)deltacount / 0.01f;

    // Save current counter value for the next calculation
    previousCounter = counter;*/


}




