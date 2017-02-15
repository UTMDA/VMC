/*******************************************************************************
 *  File Name: uart.c
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

/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "uart_CLI.h"

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

extern uint8_t CLI_rx_buf[CLI_RX_BUF_SIZE];

/* Private variables ---------------------------------------------------------*/
//Declared volatiled to prevent to be optimized away
static volatile uint8_t cnt = 0;
static uint8_t Rx_buf;

/**
 * @brief  wrapper function to do DMA transmit
 * @param  tx_buf: pointer to buffer to be send
 * @param  Size: Amount of data to be sent
 * @retval HAL Status
**/
HAL_StatusTypeDef UART_CLI_Tx(uint8_t * tx_buf, uint16_t Size)
{
    while(HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    {
        //Wait until last Tx finishes, this can be optimized for better perfomance
    }
    return HAL_UART_Transmit_DMA(&huart2, tx_buf, Size);
}

/**
 * @brief  wrapper function to do DMA receive
 * @param  Size: Amount of data to be received
 * @retval HAL Status
**/
HAL_StatusTypeDef UART_CLI_Rx(uint16_t Size)
{
    return HAL_UART_Receive_DMA(&huart2, &Rx_buf, Size);
}

/**
 * @brief  UART Tx Complete Callback function overriding _weak funcion in HAL
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
**/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

/**
 * @brief  UART Rx Complete Callback function overriding _weak funcion in HAL
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
**/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // If this an callback for CLI UART
    if (huart == &huart2)
    {
        if (Rx_buf == '\n')
        {
            //Normal Paser function
            //paser_func(cnt - 1);
            uint8_t * str = (uint8_t *)"Normal Parser Call\n";
            int i = HAL_UART_Transmit_DMA(huart,str,19);
            display("Tx Return:%d\n", i);
            cnt = 0;
        }
        else
        {
            if (cnt == CLI_RX_BUF_SIZE)
            {
                //Buffer full
                //Error Paser function that handles the overflowed buffer
                //paser_func_error();
                uint8_t * str = (uint8_t *)"\nError Parser Call\n";
                int i = HAL_UART_Transmit_DMA(huart,str,19);
                display("Tx Return:%d\n", i);
                cnt = 0;
            }
            else
            {
                //Add a new character to the buffer
                CLI_rx_buf[cnt] = Rx_buf;
                cnt++;
            }
        }
    }
}
