/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "communication.h"
#include "accelerometer.h"
#include "barometer.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Калибровочные переменные */

/* Для температуры */
///uint16_t dig_T1[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Read_Acc(double* buffer_xyz)
{
	uint8_t dev_address = 0b11010100;
  uint16_t raw_val[2];

	send_reg_log(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x28, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, 0xFF), "OUTX_L_XL");
	send_reg_log(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x29, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, 0xFF), "OUTX_H_XL");

  int16_t x_val = raw_val[1] << 8 | raw_val[0];
  raw_val[0] = raw_val[1] = 0;
  
	send_reg_log(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2A, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, 0xFF), "OUTY_L_XL");
	send_reg_log(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2B, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, 0xFF), "OUTY_H_XL");

  int16_t y_val = raw_val[1] << 8 | raw_val[0];
  raw_val[0] = raw_val[1] = 0;
  
	send_reg_log(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2C, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, 0xFF), "OUTZ_L_XL");
	send_reg_log(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2D, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, 0xFF), "OUTZ_H_XL");

  int16_t z_val = raw_val[1] << 8 | raw_val[0];

  buffer_xyz[0] = ((double)x_val * 0.488/1000)*9.81;
  buffer_xyz[1] = ((double)y_val * 0.488/1000)*9.81;
  buffer_xyz[2] = ((double)z_val * 0.488/1000)*9.81;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  char data[100] =  "F411 says: I'm alive\n\r\0";
  send_message(data, PRIORITY_HIGH);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  char str_buf[100] =  "--------------------LSM6DS33 init--------------------------\n\r";
  send_message(str_buf, PRIORITY_HIGH);

  if (check_acc_identity())
  {
		char buffer [100] = "ACCELEROMETER READ SUCCESSFULLY (nice)\n\r";
		send_message(buffer, PRIORITY_HIGH);

    acc_power_on();
  }
  else
  {
		char buffer [50] = "ACCELEROMETER READ ERROR\n\r";
		send_message(buffer, PRIORITY_HIGH);
  }

  if (check_barometer_identity())
  {
		char buffer [28] = "BMP READ SUCCESSFULLY\n\r";
		send_message(buffer, PRIORITY_HIGH);
  }
  else
  {
		char buffer [20] = "BMP READ ERROR\n\r";
		send_message(buffer, PRIORITY_HIGH);
  }

  barometer_power_on();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000);

    char data[100] =  "------------------------BMP----------------------\n\r\0";
    send_message(data, PRIORITY_HIGH);

    int32_t actual_temp = read_temp();

    char temp_str[100]; 
    sprintf(temp_str, "Temperature: %.2f Celsius\n\n\r", ((float)actual_temp)/100);
    send_message(temp_str, PRIORITY_HIGH);

    uint32_t actual_pressure = read_pressure();

    char pressure_str[100];
    sprintf(pressure_str, "Pressure: %.4f Pa\n\n\r",  ((float)actual_pressure)/256);
    send_message(pressure_str, PRIORITY_HIGH);

    char data1[100] =  "------------------------ACC----------------------\n\r\0";
    send_message(data1, PRIORITY_HIGH);

    double acc_vals[3];
    Read_Acc(acc_vals);

    char acc_str[100]; 
    sprintf(acc_str, "Acceleration: (%0.4f, %0.4f, %0.4f) \n\n\r", acc_vals[0], acc_vals[1], acc_vals[2]);
    send_message(acc_str, PRIORITY_HIGH);

    /* USER CODE END WHILE */

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
