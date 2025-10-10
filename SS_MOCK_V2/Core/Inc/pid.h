#pragma once
/*
 * pid.h
 *
 *  Created on: Sep 19, 2025
 *      Author: nicho
 */

typedef struct{

	float previous_error;

	float last_error;

	float error_integral;

//	proportional gain
	float Kp;

// 	integral gain
	float Ki;

//	derivative gain
	float Kd;

	float min_output;

	float max_output;

}PID_VARIABLES;


void init_pid (PID_VARIABLES * pid, float kp, float ki, float kd, float min_output, float max_output){

	pid ->Kp = kp;
	pid ->Ki = ki;
	pid ->Kd = kd;

	pid->error_integral = 0.0;
	pid->min_output = min_output;
    pid->max_output = max_output;

    pid->previous_error= 0.0;

}






