/*******************************************************************************
 *  File Name: led.c
 *
 *  Description: LED blink functions Source File
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         01/11/2017    Created
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "LED.h"
#include "stm32f4xx_hal.h"

/**
 * @brief  Set the LED configured in the header file
 * @param  None
 * @retval None
**/
void LED_Set(void)
{
    HAL_GPIO_WritePin(LED_USING_PORT, LED_USING_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Clear the LED configured in the header file
 * @param  None
 * @retval None
**/
void LED_Clear(void)
{
    HAL_GPIO_WritePin(LED_USING_PORT, LED_USING_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Toggle the LED configured in the header file
 * @param  None
 * @retval None
**/
void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_USING_PORT, LED_USING_PIN);
}

/**
 * @brief  Clear the LED configured in the header file according to time defined
 * @param  None
 * @retval None
**/
void LED_Blink(void)
{
    HAL_GPIO_TogglePin(LED_USING_PORT, LED_USING_PIN);
    HAL_Delay(LED_BLINK_INTERVAL/2);
    HAL_GPIO_TogglePin(LED_USING_PORT, LED_USING_PIN);
    HAL_Delay(LED_BLINK_INTERVAL/2);
}