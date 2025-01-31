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
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;

/* Для pressure */
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

int32_t actual_temp;
uint32_t actual_pressure;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))
	>> 12) *
	((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

void InitBmp()
{
	uint16_t Address_Bmp280 = 0x76 << 1; //или 0x77 - адрес устройства по линии I2C
	uint16_t Register_ID = 0xD0; //Адрес регистра в котором хранится значение ID
	uint8_t Data[1]; //Массив в котором МЫ будем хранить данные с регистра устройства
	uint16_t Size_ = 1; //Длина запрашиваемых данных, 1 байт = 1 регистр
	uint32_t Timeout_ = 0xFF; //Таймаут, 255 мс

	HAL_I2C_Mem_Read(&hi2c1, Address_Bmp280, Register_ID, I2C_MEMADD_SIZE_8BIT, Data, Size_, Timeout_);
	if (Data[0] == 0x58)
	{
		char buffer [28] = "BMP 0x58 READ SUCCESSFULLY\n\r";
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 1000);
	} else
	{
		char buffer [20] = "BMP READ ERROR\n\r";
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 1000);

	}
}

/// @brief 
/// @param status 
/// @param reg 
void UART_Send_HAL_Status(HAL_StatusTypeDef status, char* reg)
{
  char* message = NULL;

  switch (status)
  {
    case HAL_OK:
      message = "OK";
      break;
    case HAL_ERROR:
      message = "ERROR";
      break;
    
    case HAL_BUSY:
      message = "BUSY";
      break;
    case HAL_TIMEOUT:
      message = "TIMEOUT";
      break;

    default:
      break;
  }

  char buffer[100];
  strcat(buffer, reg);
  strcat(buffer, ": ");
  strcat(buffer, message);
  strcat(buffer, "\n\r\0");

  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 0xFF);
}

void Read_Dig_Variables()
{
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x88, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_T1, 2, 0xFF), "dig_T1");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x8A, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_T2, 2, 0xFF), "dig_T2");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x8C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_T3, 2, 0xFF), "dig_T3");
  
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x8E, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P1, 2, 0xFF), "dig_P1");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x90, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P2, 2, 0xFF), "dig_P2");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x92, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P3, 2, 0xFF), "dig_P3");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x94, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P4, 2, 0xFF), "dig_P4");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x96, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P5, 2, 0xFF), "dig_P5");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x98, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P6, 2, 0xFF), "dig_P6");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x9A, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P7, 2, 0xFF), "dig_P7");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x9C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P8, 2, 0xFF), "dig_P8");
  UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0x9E, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P9, 2, 0xFF), "dig_P9");
}


void Acc_LSM_Init()
{
  char str_buf[100] =  "--------------------LSM6DS33 init--------------------------\n\r";
  HAL_UART_Transmit(&huart1, (uint8_t*)str_buf, strlen(str_buf), 1000);

	uint8_t dev_address = 0b11010100; //адрес устройства по линии I2C
	uint16_t register_address = 0x0F; //Адрес регистра в котором хранится значение ID
	uint8_t data; //Массив в котором МЫ будем хранить данные с регистра устройства
	uint16_t Size_ = 1; //Длина запрашиваемых данных, 1 байт = 1 регистр
	uint32_t Timeout_ = 0xFF; //Таймаут, 255 мс

	UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, dev_address, register_address, I2C_MEMADD_SIZE_8BIT, &data, Size_, Timeout_), "WHO AM I");
	if (data == 0x69)
	{
		char buffer [100] = "ACCELEROMETER READ SUCCESSFULLY (nice)\n\r";
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 1000);


    uint8_t acc_power_mode = 0b01000100;
    UART_Send_HAL_Status(HAL_I2C_Mem_Write(&hi2c1, dev_address, 0x10, I2C_MEMADD_SIZE_8BIT, &acc_power_mode, 1, 0xFF), "ctrl_meas");
	} else
	{
		char buffer [50] = "ACCELEROMETER READ ERROR\n\r";
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 1000);
	}
}

void Read_Acc(double* buffer_xyz)
{
	uint8_t dev_address = 0b11010100;
  uint16_t raw_val[2];

	UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x28, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, 0xFF), "OUTX_L_XL");
	UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x29, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, 0xFF), "OUTX_H_XL");

  int16_t x_val = raw_val[1] << 8 | raw_val[0];
  raw_val[0] = raw_val[1] = 0;
  
	UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2A, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, 0xFF), "OUTY_L_XL");
	UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2B, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, 0xFF), "OUTY_H_XL");

  int16_t y_val = raw_val[1] << 8 | raw_val[0];
  raw_val[0] = raw_val[1] = 0;
  
	UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2C, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, 0xFF), "OUTZ_L_XL");
	UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2D, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, 0xFF), "OUTZ_H_XL");

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
    HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 1000);

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

  Acc_LSM_Init();
  InitBmp();
  Read_Dig_Variables();

  //datasheet page 25, register ctrl_meas
  uint8_t ctrl_meas = 0b01001001;
  UART_Send_HAL_Status(HAL_I2C_Mem_Write(&hi2c1, 0x76 << 1, 0xF4, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, 0xFF), "ctrl_meas");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000);

    char data[100] =  "------------------------BMP----------------------\n\r\0";
    HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 1000);

    
    uint8_t temp_raw[3];
    temp_raw[0] = temp_raw[1] = temp_raw[2]  = 0;
    UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xFA, I2C_MEMADD_SIZE_8BIT, temp_raw, 1, 0xFF), "temp_msb");
    UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xFB, I2C_MEMADD_SIZE_8BIT, temp_raw+1, 1, 0xFF), "temp_lsb");
    UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xFC, I2C_MEMADD_SIZE_8BIT, temp_raw+2, 1, 0xFF), "temp_xlsb");

    int32_t temp_raw_32 = (int32_t) ( ((uint32_t)temp_raw[0] << 12) | ((uint32_t)temp_raw[1] << 4) | ((uint32_t)temp_raw[2] >> 4) );
    actual_temp = 0;
    actual_temp = bmp280_compensate_T_int32(temp_raw_32);

    char temp_str[100]; 
    sprintf(temp_str, "Temperature: %.2f Celsius\n\n\r", ((float)actual_temp)/100);
    HAL_UART_Transmit(&huart1, (uint8_t*)temp_str, strlen(temp_str), 1000);

    uint8_t pressure_raw[3];
    UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xF7, I2C_MEMADD_SIZE_8BIT, pressure_raw, 1, 0xFF), "press_msb");
    UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xF8, I2C_MEMADD_SIZE_8BIT, pressure_raw+1, 1, 0xFF), "press_lsb");
    UART_Send_HAL_Status(HAL_I2C_Mem_Read(&hi2c1, 0x76 << 1, 0xF9, I2C_MEMADD_SIZE_8BIT, pressure_raw+2, 1, 0xFF), "press_xlsb");

    int32_t pressure_raw_32 = (int32_t) ( ((uint32_t)pressure_raw[0] << 12) | ((uint32_t)pressure_raw[1] << 4) | ((uint32_t)pressure_raw[2] >> 4) );
    actual_pressure = bmp280_compensate_P_int64(pressure_raw_32);

    char pressure_str[100];
    sprintf(pressure_str, "Pressure: %.4f Pa\n\n\r",  ((float)actual_pressure)/256);
    HAL_UART_Transmit(&huart1, (uint8_t*)pressure_str, strlen(pressure_str), 1000);

    char data1[100] =  "------------------------ACC----------------------\n\r\0";
    HAL_UART_Transmit(&huart1, (uint8_t*)data1, strlen(data), 1000);

    double acc_vals[3];
    Read_Acc(acc_vals);

    char acc_str[100]; 
    sprintf(acc_str, "Acceleration: (%0.4f, %0.4f, %0.4f) \n\n\r", acc_vals[0], acc_vals[1], acc_vals[2]);
    HAL_UART_Transmit(&huart1, (uint8_t*)acc_str, strlen(acc_str), 1000);

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
