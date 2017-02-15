/*******************************************************************************
 *  File Name: uart.h
 *
 *  Description: This file constains the wrapper functions for HAL to simplify
 *               the implementations that uses UART in other files
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         01/25/2017    Created
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UART_H
#define UART_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief  wrapper function to do DMA transmit
 * @param  tx_buf: pointer to buffer to be send
 * @param  Size: Amount of data to be sent
 * @retval HAL Status
**/
HAL_StatusTypeDef UART_CLI_Tx(uint8_t * tx_buf, uint16_t Size);

/**
 * @brief  wrapper function to do DMA receive
 * @param  Size: Amount of data to be received
 * @retval HAL Status
**/
HAL_StatusTypeDef UART_CLI_Rx(uint16_t Size);

/**
 * @brief  UART Tx Complete Callback function overriding _weak funcion in HAL
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
**/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief  UART Rx Complete Callback function overriding _weak funcion in HAL
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
**/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* UART_H */