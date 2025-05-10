/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
typedef struct
{
  int16_t  AC1, AC2, AC3;
  uint16_t AC4, AC5, AC6;
  int16_t  B1, B2;
  int32_t B5;
  int16_t  MB, MC, MD;
} BP_BMP180_Calibration_Data;

typedef struct
{
  int32_t raw_temp;
  int32_t raw_pressure;
  float temp;
  float altitude;
  float takeoff_altitude;
  float relative_altitude;
} BP_BMP180_Dev;

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

HAL_StatusTypeDef BP_Check_BMI270_Ready();
HAL_StatusTypeDef BP_Check_BMP180_Ready();
int8_t BP_BMI270_I2C_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t BP_BMI270_I2C_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
HAL_StatusTypeDef BP_BMP180_Read_Calibration_Data(BP_BMP180_Calibration_Data *calib_data);
int32_t BP_BMP180_Get_Temp(BP_BMP180_Calibration_Data *calib_data);
int32_t BP_BMP180_Get_Pressure(BP_BMP180_Calibration_Data *calib_data, uint8_t oss);
float BP_BMP180_Calculate_Altitude(int32_t pressure_pa, int32_t sea_level_pa);
void BP_BMP180_Calibrate_Ground_Level(BP_BMP180_Calibration_Data *calib_data, BP_BMP180_Dev *bmp_180_dev);
void BP_BMP180_Get_Data(BP_BMP180_Calibration_Data *calib_data, BP_BMP180_Dev *bmp_180_dev);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

