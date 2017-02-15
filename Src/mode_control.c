/*******************************************************************************
 *  File Name: mode_control.c
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

/* Includes ------------------------------------------------------------------*/
#include "mode_control.h"
#include "led.h"

/* Private variables ---------------------------------------------------------*/
//Declared volatiled to prevent to be optimized away
static volatile uint8_t mode_control;

static uint16_t adc_upper_limit;
static uint16_t adc_lower_limit;

static ADC_HandleTypeDef * adc;
static TIM_HandleTypeDef * tim;

/* Private function prototypes -----------------------------------------------*/
static void set_mode(void);

/**
 * @brief  Start the mode control function, pre-set the adc limits
 * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *         the configuration information for TIM module.
 * @retval None
**/
void Mode_Control_Start(ADC_HandleTypeDef * hadc, TIM_HandleTypeDef * htim)
{
    adc = hadc;
    tim = htim;
    HAL_TIM_Base_Start(tim);
    HAL_ADC_Start_IT(adc);
    //Set adc_upper_limit and adc_lower_limit
}

/**
 * @brief  Regular conversion complete callback in non blocking mode 
 * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
**/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint16_t value = HAL_ADC_GetValue(hadc);
    if(value >= adc_lower_limit && value <= adc_upper_limit)
    {
        // Only change the mode if it not currently in force mode
        if(!(mode_control >> 15))
        {
            mode_control = RADIO_MODE;
            set_mode();
        }
    }
    else
    {
        // Only change the mode if it not currently in force mode
        if(!(mode_control >> 15))
        {
            mode_control = VMC_MODE;
            set_mode();
        }
    }
}

/**
 * @brief  Helper function that set analog switch to radio/VMC
 * @param  None
 * @retval None
**/
static void set_mode(void)
{
    //write XXX to GPIO
}