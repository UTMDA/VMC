/*******************************************************************************
 *  File Name: uart_CLI.h
 *
 *  Description: This file constains the key functions that enables a command
 *               line interface for in-field debugging
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         01/23/2017    Created
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UART_CLI_H
#define UART_CLI_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* CLI defines ---------------------------------------------------------------*/
#define CLI_TX_BUF_SIZE 32
#define CLI_RX_BUF_SIZE 32
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief  Initialize the UART Command Line Interface
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
**/
void UART_CLI_Init(UART_HandleTypeDef * huart);

#endif /* UART_CLI_H */