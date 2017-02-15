/*******************************************************************************
 *  File Name: motor_control.c
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

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"

/**
 * @brief  Start the turning motor
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
**/
void Motor_Start(TIM_HandleTypeDef * htim)
{
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, MOTOR_PWM_NEUTRAL_DUTY_CYCLE);
}

/**
 * @brief  Stop the turning motor
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
**/
void Motor_Stop(TIM_HandleTypeDef * htim)
{
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
}