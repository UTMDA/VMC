/*******************************************************************************
 *  File Name: _9_axis.c
 *
 *  Description: This file constains the key functions that works with BNO055
 *               9-axis sensosr
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         01/16/2017    Created
 *    Shizhang Yin         01/21/2017    Added functions that are required for
 *                                       BNO055 driver to work
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "_9_axis.h"

/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef * i2c;
static struct bno055_t bno055;

/**
 * @brief  Initialize the BNO055 device and setup I2C
 * @param  hi2c: Pointer to I2C_HandleTypeDef structure that contains
 *              the configuration information for the specified I2C module
 * @retval None
**/
void _9_Axis_Init(I2C_HandleTypeDef * hi2c)
{
    i2c = hi2c;

    bno055.dev_addr = _9_AXIS_ADDRESS;
    bno055.bus_write = _9_Axis_Bus_Write;
    bno055.bus_read = _9_Axis_Bus_Read;
    bno055.delay_msec = Delay_Func;

    if(bno055_init(&bno055) != BNO055_SUCCESS)
    {
        display("BNO055 initialization failed!");
    }
    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
}

/**
* @brief  This function will be called with the BNO055 API
**/
BNO055_RETURN_FUNCTION_TYPE _9_Axis_Bus_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t * reg_data, uint8_t wr_len)
{
    uint8_t array[I2C_BUFFER_LEN];
    array[0] = reg_addr; // Set the first element in the array to be the address of the register in slave
    for (uint8_t i = 0; i < wr_len; i++)
    {
        array[i + 1] = *(reg_data + i); // Offset the copy by one to leave space for address of register in slave
    }
    if (HAL_I2C_Master_Transmit(i2c, dev_addr, array, wr_len + 1, I2C_TIMEOUT) == HAL_OK)
        return BNO055_SUCCESS;
    else
        return BNO055_ERROR;
}

/**
* @brief  This function will be called with the BNO055 API
**/
BNO055_RETURN_FUNCTION_TYPE _9_Axis_Bus_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t * reg_data, uint8_t r_len)
{
    if (HAL_I2C_Master_Transmit(i2c, dev_addr, &reg_addr, 1, I2C_TIMEOUT) == HAL_OK) // Transmit the address that you want to read from
    {
        if(HAL_I2C_Master_Receive(i2c, dev_addr, reg_data, r_len, I2C_TIMEOUT) == HAL_OK) // Receive data
            return BNO055_SUCCESS;
        else
            return BNO055_ERROR;
    }
    return BNO055_ERROR;
}

/**
* @brief  This function will be called with the BNO055 API
**/
BNO055_DELAY_RETURN_TYPE Delay_Func(BNO055_DELAY_PARAM_TYPES delay_in_msec)
{
    HAL_Delay(delay_in_msec);
}
