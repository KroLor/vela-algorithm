#include "barometer.h"
#include "communication.h"
#include <i2c.h>

static const uint8_t dev_address = 0x76 << 1; // или 0x77, адрес устройства по линии I2C
static const uint32_t timeout_default = 0xFF; // Таймаут, 255 мс

static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;

/* Для pressure */
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t bmp280_compensate_T_int32(int32_t adc_T);

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P_int64(int32_t adc_P);

uint32_t read_pressure()
{
	uint8_t pressure_raw[3];
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0xF7, I2C_MEMADD_SIZE_8BIT, pressure_raw, 1, timeout_default), "press_msb", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0xF8, I2C_MEMADD_SIZE_8BIT, pressure_raw + 1, 1, timeout_default), "press_lsb", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0xF9, I2C_MEMADD_SIZE_8BIT, pressure_raw + 2, 1, timeout_default), "press_xlsb", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);

	int32_t pressure_raw_32 = (int32_t)(((uint32_t)pressure_raw[0] << 12) | ((uint32_t)pressure_raw[1] << 4) | ((uint32_t)pressure_raw[2] >> 4));
	uint32_t actual_pressure = bmp280_compensate_P_int64(pressure_raw_32);
	return actual_pressure;
}

int32_t read_temp()
{
	uint8_t temp_raw[3];
	temp_raw[0] = temp_raw[1] = temp_raw[2] = 0;
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0xFA, I2C_MEMADD_SIZE_8BIT, temp_raw, 1, timeout_default), "temp_msb", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0xFB, I2C_MEMADD_SIZE_8BIT, temp_raw + 1, 1, timeout_default), "temp_lsb", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0xFC, I2C_MEMADD_SIZE_8BIT, temp_raw + 2, 1, timeout_default), "temp_xlsb", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);

	int32_t temp_raw_32 = (int32_t)(((uint32_t)temp_raw[0] << 12) | ((uint32_t)temp_raw[1] << 4) | ((uint32_t)temp_raw[2] >> 4));
	int32_t actual_temp = bmp280_compensate_T_int32(temp_raw_32);
	return actual_temp;
}

short check_barometer_identity()
{
	uint16_t Register_ID = 0xD0; // Адрес регистра в котором хранится значение ID
	uint8_t Data[1];			 // Массив в котором МЫ будем хранить данные с регистра устройства
	uint16_t Size_ = 1;			 // Длина запрашиваемых данных, 1 байт = 1 регистр

	HAL_I2C_Mem_Read(&hi2c1, dev_address, Register_ID, I2C_MEMADD_SIZE_8BIT, Data, Size_, timeout_default);
	if (Data[0] == 0x58)
	{
		return 1;
	}
	else
	{
		return 0;
	}

	return 0;
}

void read_dig_variables()
{
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x88, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_T1, 2, timeout_default), "dig_T1", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x8A, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_T2, 2, timeout_default), "dig_T2", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x8C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_T3, 2, timeout_default), "dig_T3", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);

	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x8E, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P1, 2, timeout_default), "dig_P1", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x90, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P2, 2, timeout_default), "dig_P2", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x92, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P3, 2, timeout_default), "dig_P3", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x94, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P4, 2, timeout_default), "dig_P4", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x96, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P5, 2, timeout_default), "dig_P5", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x98, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P6, 2, timeout_default), "dig_P6", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x9A, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P7, 2, timeout_default), "dig_P7", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x9C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P8, 2, timeout_default), "dig_P8", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x9E, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dig_P9, 2, timeout_default), "dig_P9", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
}

short barometer_power_on()
{
	read_dig_variables();

	// datasheet page 25, register ctrl_meas
	uint8_t ctrl_meas = 0b01001001;
	log_register(HAL_I2C_Mem_Write(&hi2c1, dev_address, 0xF4, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, timeout_default), "ctrl_meas", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);

	return 0;
}

static int32_t t_fine;
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
			((int32_t)dig_T3)) >>
		   14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	return (uint32_t)p;
}