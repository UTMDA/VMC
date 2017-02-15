/*******************************************************************************
 *  File Name: uart_CLI.c
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

/* Includes ------------------------------------------------------------------*/
#include "uart_CLI.h"

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef * uart;

/* Shared variables ----------------------------------------------------------*/
uint8_t CLI_tx_buf[CLI_TX_BUF_SIZE];
uint8_t CLI_rx_buf[CLI_RX_BUF_SIZE];

/**
 * @brief  Initialize the BNO055 device and setup I2C
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
**/
void UART_CLI_Init(UART_HandleTypeDef * huart)
{
    uart = huart;
    /* Start CLI */
    UART_CLI_Rx(1);
}