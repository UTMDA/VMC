/*******************************************************************************
 *  File Name: main.h
 *
 *  Description: Main program Header File
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         01/11/2017    Created
 *    Shizhang Yin         01/13/2017    Added Servo definitions
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

/* Includes ------------------------------------------------------------------*/

/* Pin defines ---------------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* PWM configurations --------------------------------------------------------*/
#define PWM_RESOLUTION 1000
#define PWM_FREQUENCY 100 // In Hz
#define SERVO_PWM_DEFAULT_DUTY_CYCLE 0
#define SERVO_PWM_NEUTRAL_DUTY_CYCLE 150
#define SERVO_PWM_MIN_DUTY_CYCLE 110
#define SERVO_PWM_MAX_DUTY_CYCLE 190
#define MOTOR_PWM_DEFAULT_DUTY_CYCLE 0
#define MOTOR_PWM_NEUTRAL_DUTY_CYCLE 150
#define MOTOR_PWM_MIN_DUTY_CYCLE 110
#define MOTOR_PWM_MAX_DUTY_CYCLE 190

#endif /* MAIN_H */