/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "bmi270.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define GRAVITY_EARTH  (9.80665f)
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct bmi2_dev bmi;
uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
struct bmi2_sens_config sens_config[2];
struct bmi2_sens_data sensor_data = { { 0 } };
float acc_x, acc_y, acc_z;
float gyr_x, gyr_y, gyr_z;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
extern void initialise_monitor_handles(void);
void BP_Delay_us(uint32_t period_us, void *intf_ptr);
void BP_Enable_DWT_Cycle_Counter(void);
int8_t BP_BMI270_Init(void);
int8_t BP_BMI270_Accel_Gyro_Config(struct bmi2_dev *bmi);
void BP_BMI270_Get_Sensor_Data();
float BP_LSB_To_MPS(int16_t val, float g_range, uint8_t bit_width);
float BP_LSB_To_DPS(int16_t val, float dps, uint8_t bit_width);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t BP_BMI270_Init(void) 
{
  int8_t result;
  bmi.intf_ptr = &hi2c1;
  bmi.chip_id = BMI270_CHIP_ID;
  bmi.intf = BMI2_I2C_INTF;
  bmi.read = BP_BMI270_I2C_Read;
  bmi.write = BP_BMI270_I2C_Write;
  bmi.delay_us = BP_Delay_us;
  bmi.read_write_len = 64;
  bmi.config_file_ptr = NULL;

  result = bmi270_init(&bmi);
  if (result == BMI2_OK) 
  {
    printf("BMI270 INIT \n");
  } else {
    printf("BMI270 FAILED INIT %d \n", result);
  }

  return result;
}

int8_t BP_BMI270_Accel_Gyro_Config(struct bmi2_dev *bmi) 
{
  int8_t result;
  
  sens_config[ACCEL].type = BMI2_ACCEL;
  sens_config[GYRO].type = BMI2_GYRO;

  result = bmi2_get_sensor_config(sens_config, 2, bmi);
  if (result == BMI2_OK) 
  {
    sens_config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    sens_config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;
    sens_config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    sens_config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    
    sens_config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    sens_config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    sens_config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    sens_config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    result = bmi2_set_sensor_config(sens_config, 2, bmi);

    if (result == BMI2_OK)
    {
      result = bmi270_sensor_enable(sensor_list, 2, bmi);
      if (result == BMI2_OK)
      {
        printf("SENSOR SET AND ENABLE SUCCESS \n");
      } else {
        printf("SENSOR SET AND ENABLE FAILED \n");
      }
    } else {
      printf("SET SENSOR CONFIG FAILED \n");
    }
  } else {
    printf("GET SENSOR CONFIG FAILED \n");
  }

  return result;
}

void BP_BMI270_Get_Sensor_Data() 
{
  int8_t result;
  result = bmi2_get_sensor_data(&sensor_data, &bmi);
  if (result == BMI2_OK && (sensor_data.status & BMI2_DRDY_ACC) && (sensor_data.status & BMI2_DRDY_GYR)) 
  {
    acc_x = BP_LSB_To_MPS(sensor_data.acc.x, (float)2, bmi.resolution);
    acc_y = BP_LSB_To_MPS(sensor_data.acc.y, (float)2, bmi.resolution);
    acc_z = BP_LSB_To_MPS(sensor_data.acc.z, (float)2, bmi.resolution);

    gyr_x = BP_LSB_To_DPS(sensor_data.gyr.x, (float)2000, bmi.resolution);
    gyr_y = BP_LSB_To_DPS(sensor_data.gyr.y, (float)2000, bmi.resolution);
    gyr_z = BP_LSB_To_DPS(sensor_data.gyr.z, (float)2000, bmi.resolution);
  } else {
    printf("GET SENSOR DATA FAILED \n");
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  initialise_monitor_handles();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  BP_Enable_DWT_Cycle_Counter();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  int8_t bmi270_result = BP_BMI270_Init();
  if (bmi270_result == BMI2_OK) 
  {
    BP_BMI270_Accel_Gyro_Config(&bmi);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    BP_BMI270_Get_Sensor_Data();
    printf("ACCEL X: %.3f, ACCEL Y: %.3f, ACCEL Z: %.3f | GYRO X: %.3f, GYRO Y: %.3f, GYRO X: %.3f \n", acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);
    HAL_Delay(50);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*!
  @brief Delay setup for BMI270 dev struct
*/
void BP_Delay_us(uint32_t period_us, void *intf_ptr) 
{
  (void)intf_ptr;
  #if (__CORTEX_M >= 0x03U)
  uint32_t start_tick = DWT->CYCCNT;
  uint32_t delay_ticks = period_us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start_tick) < delay_ticks);
  #else
  volatile uint32_t count = period_us * (SystemCoreClock / 1000000U / 4U); // Adjust divisor
  while (count > 0) {
      count--;
      __NOP(); // No operation, helps prevent optimization out
  }
  if (period_us == 0) { // Ensure at least a minimal delay if 0 is passed
      __NOP();
  }
  #endif
}

/*!
  @brief Enable microseconds clock counter for ARM Cortext M3
*/
void BP_Enable_DWT_Cycle_Counter(void) 
{
  #if (__CORTEX_M >= 0x03U)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  #endif
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
float BP_LSB_To_MPS(int16_t val, float g_range, uint8_t bit_width)
{
  double power = 2;
  float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
  return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
float BP_LSB_To_DPS(int16_t val, float dps, uint8_t bit_width)
{
  double power = 2;
  float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
  return (dps / (half_scale)) * (val);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
