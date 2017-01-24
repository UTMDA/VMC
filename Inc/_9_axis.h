/*******************************************************************************
 *  File Name: _9_axis.h
 *
 *  Description: This file constains the key functions that works with BNO055
                 9-axis sensosr
 *
 *    History:
 *    Author               Date          Description
 *    --------------------------------------------------------------------------
 *    Shizhang Yin         01/16/2017    Created
 *
 * Copyright (c) UTMDA, 2017
 * Licensed under the MIT License. See LICENSE file in the project root for full
 * license information.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _9_AXIS_H
#define _9_AXIS_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bno055.h"
#include "stm32f4xx_hal.h"

/* 9-axis defines ------------------------------------------------------------*/
#define _9_AXIS_ADDRESS (uint8_t)(BNO055_I2C_ADDR1 << 1)
#define I2C_TIMEOUT 100
#define	I2C_BUFFER_LEN 8

/* Function prototypes -------------------------------------------------------*/
void _9_Axis_Init(I2C_HandleTypeDef * hi2c);

BNO055_RETURN_FUNCTION_TYPE _9_Axis_Bus_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t * reg_data, uint8_t r_len);
BNO055_RETURN_FUNCTION_TYPE _9_Axis_Bus_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t * reg_data, uint8_t wr_len);
BNO055_DELAY_RETURN_TYPE Delay_Func(BNO055_DELAY_PARAM_TYPES delay_in_msec);

#endif /* _9_AXIS_H */