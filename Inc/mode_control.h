/*******************************************************************************
 *  File Name: mode_control.h
 *
 *  Description: This file constains the functions that decide if the vehicle is
 *               under radio or computer control
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         02/13/2017    Created
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODE_CONTROL_H
#define MODE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Mode defines --------------------------------------------------------------*/
#define STOP_MODE           (uint8_t) 0x00
#define RADIO_MODE          (uint8_t) 0x01
#define VMC_MODE            (uint8_t) 0x02
#define FORCE_RADIO_MODE    (uint8_t) 0xFD
#define FORCE_VMC_MODE      (uint8_t) 0xFE
#define FORCE_EXIT          (uint8_t) 0xFF

/**
 * @brief  Start the mode control function, pre-set the adc limits
 * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *         the configuration information for TIM module.
 * @retval None
**/
void Mode_Control_Start(ADC_HandleTypeDef * hadc, TIM_HandleTypeDef * htim);

/**
 * @brief  Regular conversion complete callback in non blocking mode 
 * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
**/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif