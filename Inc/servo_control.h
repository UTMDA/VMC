/*******************************************************************************
 *  File Name: servo_control.h
 *
 *  Description: generate required PWM signals to control the turning servo
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         01/13/2017    Created
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Function prototypes -------------------------------------------------------*/
void Servo_Start(TIM_HandleTypeDef * htim);
void Servo_Stop(TIM_HandleTypeDef * htim);

#endif /* SERVO_CONTROL_H */