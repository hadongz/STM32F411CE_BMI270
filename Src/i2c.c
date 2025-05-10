/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "bmi2_defs.h"

/* USER CODE BEGIN 0 */
#define BMI270_ADDRESS UINT8_C(0x69)

#define BMP180_ADDRESS UINT8_C(0xEE)
#define BMP180_CALIBRATION_VALUE_LENGTH UINT8_C(22)
#define BMP180_CALIBRATION_START_ADDRESS UINT8_C(0xAA)
#define BMP180_I2C_TIMEOUT UINT32_C(1000)
#define BMP180_CONTROL_WRITE_ADDRESS UINT8_C(0xF4)
#define BMP180_CONTROL_READ_ADDRESS UINT8_C(0xF6)
#define BMP180_READ_ERROR INT32_C(0x80000000)

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef BP_Check_BMI270_Ready() 
{
  HAL_StatusTypeDef status;
  uint16_t dev_address = (BMI270_ADDRESS << 1);
  status = HAL_I2C_IsDeviceReady(&hi2c1, dev_address, 10, HAL_MAX_DELAY);
  return status;
}

HAL_StatusTypeDef BP_Check_BMP180_Ready() 
{
  HAL_StatusTypeDef status;
  status = HAL_I2C_IsDeviceReady(&hi2c1, BMP180_ADDRESS, 10, HAL_MAX_DELAY);
  return status;
 }

 int8_t BP_BMI270_I2C_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if (intf_ptr == NULL) {
    return BMI2_E_NULL_PTR;
  }

  I2C_HandleTypeDef *h12c = (I2C_HandleTypeDef *) intf_ptr;
  uint8_t shifted_addr = (BMI270_ADDRESS << 1);

  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(h12c, shifted_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY);

  if (status == HAL_OK) {
    return BMI2_OK;
  } else {
    return BMI2_E_COM_FAIL;
  }
}

int8_t BP_BMI270_I2C_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) 
{
  if (intf_ptr == NULL) {
    return BMI2_E_NULL_PTR;
  }

  I2C_HandleTypeDef *h12c = (I2C_HandleTypeDef *) intf_ptr;
  uint8_t shifted_addr = (BMI270_ADDRESS << 1);

  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Write(h12c, shifted_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *) reg_data, len, HAL_MAX_DELAY);

  if (status == HAL_OK) {
    return BMI2_OK;
  } else {
    return BMI2_E_COM_FAIL;
  }
}

HAL_StatusTypeDef BP_BMP180_Read_Calibration_Data(BP_BMP180_Calibration_Data *calib_data)
{
  HAL_StatusTypeDef result;
  uint8_t cal_buffer[BMP180_CALIBRATION_VALUE_LENGTH];
  result = HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, BMP180_CALIBRATION_START_ADDRESS, I2C_MEMADD_SIZE_8BIT, cal_buffer, BMP180_CALIBRATION_VALUE_LENGTH, HAL_MAX_DELAY);

  if (result == HAL_OK) 
  {
    calib_data->AC1 = (int16_t)((cal_buffer[0] << 8) | cal_buffer[1]);
    calib_data->AC2 = (int16_t)((cal_buffer[2] << 8) | cal_buffer[3]);
    calib_data->AC3 = (int16_t)((cal_buffer[4] << 8) | cal_buffer[5]);
    calib_data->AC4 = (uint16_t)((cal_buffer[6] << 8) | cal_buffer[7]);
    calib_data->AC5 = (uint16_t)((cal_buffer[8] << 8) | cal_buffer[9]);
    calib_data->AC6 = (uint16_t)((cal_buffer[10] << 8) | cal_buffer[11]);
    calib_data->B1 = (int16_t)((cal_buffer[12] << 8) | cal_buffer[13]);
    calib_data->B2 = (int16_t)((cal_buffer[14] << 8) | cal_buffer[15]);
    calib_data->MB = (int16_t)((cal_buffer[16] << 8) | cal_buffer[17]);
    calib_data->MC = (int16_t)((cal_buffer[18] << 8) | cal_buffer[19]);
    calib_data->MD = (int16_t)((cal_buffer[20] << 8) | cal_buffer[21]);
    printf("BMP180 SUCCESS READING CALIBRATION DATA\n");
  } else {
    printf("BMP180 FAILED READING CALIBRATION DATA\n");
  }

  return result;
}

HAL_StatusTypeDef BP_BMP180_Read_Register(uint8_t reg_addr, uint8_t *reg_data)
{
  return HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, 1, BMP180_I2C_TIMEOUT);
}

HAL_StatusTypeDef BP_BMP180_Write_Register(uint8_t reg_addr, uint8_t reg_data)
{
  return HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, BMP180_I2C_TIMEOUT);
}

HAL_StatusTypeDef BP_BMP180_Read_Multiple_Registers(uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, BMP180_I2C_TIMEOUT);
}

int32_t BP_BMP180_Get_Temp(BP_BMP180_Calibration_Data *calib_data)
{
  HAL_StatusTypeDef status;
  uint8_t data_to_write = 0x2E;
  uint8_t raw_data[2] = {0};
  status = BP_BMP180_Write_Register(BMP180_CONTROL_WRITE_ADDRESS, data_to_write);
  if(status == HAL_OK)
  {
    HAL_Delay (5);
    status = BP_BMP180_Read_Multiple_Registers(BMP180_CONTROL_READ_ADDRESS, raw_data, 2);
    if(status == HAL_OK)
    {
      /*!
        @brief  BMP180 Data sheet page 15
      */
      int32_t UT = ((raw_data[0] << 8) | raw_data[1]);
      int32_t X1 = (((UT - (int32_t)calib_data->AC6)) * (int32_t)calib_data->AC5) >> 15;
      int32_t X2 = ((int32_t)calib_data->MC << 11) / (X1 + (int32_t)calib_data->MD);
      int32_t B5 = X1 + X2;
      calib_data->B5 = B5;
      int32_t temp = (B5 + 8) >> 4;
      return temp;
    } else {
      printf("FAILED TO READ REGISTER UNCOMPENSTATED TEMP \n");
    }
  } else {
    printf("FAILED TO WRITE REGISTER UNCOMPENSTATED TEMP \n");
  }

  return BMP180_READ_ERROR;
}

int32_t BP_BMP180_Get_Pressure(BP_BMP180_Calibration_Data *calib_data, uint8_t oss)
{
  HAL_StatusTypeDef status;
  uint8_t data_to_write = 0x34 + (oss << 6);
  uint8_t raw_data[3] = {0};
  uint32_t wait_time_ms;

  switch(oss)
  {
    case 0: wait_time_ms = 5; break;
    case 1: wait_time_ms = 8; break;
    case 2: wait_time_ms = 14; break;
    case 3: wait_time_ms = 26; break;
    default: wait_time_ms = 5; break;
  }

  status = BP_BMP180_Write_Register(BMP180_CONTROL_WRITE_ADDRESS, data_to_write);
  if (status == HAL_OK)
  {
    HAL_Delay(wait_time_ms);
    status = BP_BMP180_Read_Multiple_Registers(BMP180_CONTROL_READ_ADDRESS, raw_data, 3);
    if (status == HAL_OK)
    {
      /*!
        @brief  BMP180 Data sheet page 15
      */
      int32_t UP, B6, X1, X2, X3, B3;
      uint32_t B4, B7;
      int32_t p;
      
      // Proper way to read uncompensated pressure according to datasheet
      UP = ((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]) >> (8 - oss);
      
      // Make sure we have valid temperature compensation data first
      if (calib_data->B5 == 0) {
        BP_BMP180_Get_Temp(calib_data);  // Get temperature first to calculate B5
      }
      
      B6 = calib_data->B5 - 4000;
      X1 = ((int32_t)calib_data->B2 * ((B6 * B6) >> 12)) >> 11;
      X2 = ((int32_t)calib_data->AC2 * B6) >> 11;
      X3 = X1 + X2;
      B3 = ((((int32_t)calib_data->AC1 * 4 + X3) << oss) + 2) >> 2;

      X1 = ((int32_t)calib_data->AC3 * B6) >> 13;
      X2 = ((int32_t)calib_data->B1 * ((B6 * B6) >> 12)) >> 16;
      X3 = ((X1 + X2) + 2) >> 2;
      B4 = ((uint32_t)calib_data->AC4 * (uint32_t)(X3 + 32768)) >> 15;
      B7 = ((uint32_t)UP - (uint32_t)B3) * (50000UL >> oss);

      if (B7 < 0x80000000)
      {
        p = (B7 * 2) / B4;
      } else {
        p = (B7 / B4) * 2;
      }

      X1 = (p >> 8) * (p >> 8);
      X1 = (X1 * 3038) >> 16;
      X2 = (-7357 * p) >> 16;
      p = p + ((X1 + X2 + 3791) >> 4);

      return p;
    } else {
      printf("FAILED TO READ REGISTER UNCOMPENSATED PRESSURE \n");
    }
  } else {
    printf("FAILED TO WRITE REGISTER UNCOMPENSATED PRESSURE \n");
  }

  return BMP180_READ_ERROR;
}

float BP_BMP180_Calculate_Altitude(int32_t pressure_pa, int32_t sea_level_pa)
{
    // If no sea level pressure provided, use standard atmospheric pressure
    if (sea_level_pa == 0) {
        sea_level_pa = 101325; // 1013.25 hPa in Pa
    }
    
    // Altitude calculation using the international barometric formula
    // h = 44330 * (1 - (P/P0)^(1/5.255))
    float pressure_ratio = (float)pressure_pa / (float)sea_level_pa;
    float altitude = 44330.0f * (1.0f - powf(pressure_ratio, 0.1902949f)); // 1/5.255 = 0.1902949

    return altitude;
}

void BP_BMP180_Calibrate_Ground_Level(BP_BMP180_Calibration_Data *calib_data, BP_BMP180_Dev *bmp_180_dev) 
{
  BP_BMP180_Get_Temp(calib_data);
  int32_t current_pressure = BP_BMP180_Get_Pressure(calib_data, 3);
  bmp_180_dev->takeoff_altitude = BP_BMP180_Calculate_Altitude(current_pressure, 0);
  bmp_180_dev->raw_pressure = current_pressure;
}

void BP_BMP180_Get_Data(BP_BMP180_Calibration_Data *calib_data, BP_BMP180_Dev *bmp_180_dev)
{
  int32_t raw_temp = BP_BMP180_Get_Temp(calib_data);
  int32_t raw_pressure = BP_BMP180_Get_Pressure(calib_data, 3);
  float temp = (float)raw_temp / 10;
  float altitude = BP_BMP180_Calculate_Altitude(raw_pressure, 0);
  float relative_altitude = altitude - bmp_180_dev->takeoff_altitude;
  
  bmp_180_dev->raw_temp = raw_temp;
  bmp_180_dev->raw_pressure = raw_pressure;
  bmp_180_dev->temp = temp;
  bmp_180_dev->altitude = altitude;
  bmp_180_dev->relative_altitude = relative_altitude;
}

/* USER CODE END 1 */
