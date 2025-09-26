/*
 * motordrivers.h
 *
 *  Created on: Sep 12, 2025
 *      Author: nicho, adam :p
 */
/**
 * Motor driving abstractions.
 */

#pragma once

#include "stm32l4xx_hal.h"

/*
 * Default motor scale limits.
 */


/**
 * Represents a timer PWM driver.
 */
typedef struct
{
    /**
     * ST TIM handler.
     */
    TIM_HandleTypeDef *timer;

    /**
     * The timer channel to output on.
     */
    uint32_t channel;

    /**
     * The timer period.
     */
    uint32_t period;
} PWM_Driver;

/**
 * Represents a motor.
 */
typedef struct
{
    /**
     * The timer PWM for Motor.
     */
    PWM_Driver Motor_t;

    /**
     * the Direction Port and pin GPIO port
     */
    GPIO_TypeDef *DirPort;


    uint16_t DirPin;

    /**
     * The maximum forward driving power scale.
     */
    float max_fwd;

    /**
     * The maximum reverse driving power scale.
     */
    float max_back;
} Motor;

//rw - reactionwheel
void init_default_motor(Motor *RW,
                        TIM_HandleTypeDef *pwm_tim,
                        uint32_t pwm_channel,
                        GPIO_TypeDef *Dir_port,
                        uint16_t Dir_pin);

/**
 * Drive the PWM of the motor.
 *
 * @param[in] motor a pointer to the motor object to drive.
 * @param[in] power the duty cycle and direction to drive.
 *
 */
void drive_motor(Motor *RW, float power);
