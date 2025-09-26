#pragma once
/*
 * pid.h
 *
 *  Created on: Sep 19, 2025
 *      Author: nicho
 */

typedef struct{

	float error;
	//current - desired = error

	float desired_pos;

	float current_pos;

	float Kp;

	float Ki;

	float Kd;


}PID_VARIABLES;





