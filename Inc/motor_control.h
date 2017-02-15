/*******************************************************************************
 *  File Name: motor_control.h
 *
 *  Description: generate required PWM signals to control the turning motor
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         01/30/2017    Created
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Start the turning motor
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
**/
void Motor_Start(TIM_HandleTypeDef * htim);

/**
 * @brief  Stop the turning motor
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
**/
void Motor_Stop(TIM_HandleTypeDef * htim);

#endif /* MOTOR_CONTROL_H */