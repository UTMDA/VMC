/*******************************************************************************
 *  File Name: LED.h
 *
 *  Description: LED blink functions Header File
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_H
#define LED_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* LED defines ---------------------------------------------------------------*/
#define LED_USING_PORT LD2_GPIO_Port
#define LED_USING_PIN LD2_Pin

/* Time defines --------------------------------------------------------------*/
#define LED_BLINK_INTERVAL 1000 // This number is in milliseconds

/* Function prototypes -------------------------------------------------------*/
void LED_Set(void);
void LED_Clear(void);
void LED_Toggle(void);
void LED_Blink(void);

#endif /* LED_H */