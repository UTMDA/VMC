/*******************************************************************************
 *  File Name: servo_control.c
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

/* Includes ------------------------------------------------------------------*/
#include "servo_control.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Start the turning servo
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
**/
void Servo_Start(TIM_HandleTypeDef * htim)
{
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

/**
 * @brief Stop the turning servo
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
**/
void Servo_Stop(TIM_HandleTypeDef * htim)
{
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
}