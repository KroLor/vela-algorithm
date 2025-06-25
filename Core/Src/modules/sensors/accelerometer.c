#include "accelerometer.h"
#include <stdint.h>
#include "i2c.h"
#include "usart.h"
#include "communication.h"
#include "string.h"

static const uint8_t dev_address = 0b11010100; //адрес устройства по линии I2C
static const uint32_t timeout_default = 0xFF; //Таймаут, 255 мс

short check_acc_identity()
{
	uint16_t register_address = 0x0F; //Адрес регистра в котором хранится значение ID
	uint8_t data; //Массив в котором МЫ будем хранить данные с регистра устройства
	uint16_t Size_ = 1; //Длина запрашиваемых данных, 1 байт = 1 регистр

	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, register_address, I2C_MEMADD_SIZE_8BIT, &data, Size_, timeout_default), "WHO AM I", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	if (data == 0x69)
	{
		//successfuly read register
		return 1;
	} else
	{
		char buffer [50] = "ACCELEROMETER READ ERROR\n\r";
		send_message(buffer, PRIORITY_HIGH);
		return 0;
	}
}

short acc_power_on()
{
	uint8_t acc_power_mode = 0b01000100;
	log_register(HAL_I2C_Mem_Write(&hi2c1, dev_address, 0x10, I2C_MEMADD_SIZE_8BIT, &acc_power_mode, 1, timeout_default), "ctrl_meas", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);

	return 0;
}

void read_acceleration_xyz(double* buffer_xyz)
{
	uint16_t raw_val[2];

	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x28, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, timeout_default), "OUTX_L_XL", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x29, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, timeout_default), "OUTX_H_XL", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);

	int16_t x_val = raw_val[1] << 8 | raw_val[0];
	raw_val[0] = raw_val[1] = 0;

	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2A, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, timeout_default), "OUTY_L_XL", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2B, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, timeout_default), "OUTY_H_XL", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);

	int16_t y_val = raw_val[1] << 8 | raw_val[0];
	raw_val[0] = raw_val[1] = 0;

	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2C, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val, 1, timeout_default), "OUTZ_L_XL", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);
	log_register(HAL_I2C_Mem_Read(&hi2c1, dev_address, 0x2D, I2C_MEMADD_SIZE_8BIT, (uint8_t*)raw_val+1, 1, timeout_default), "OUTZ_H_XL", SYS_STATE_NONE, SYS_AREA_PERIPH_ACC);

	int16_t z_val = raw_val[1] << 8 | raw_val[0];

	buffer_xyz[0] = ((double)x_val * 0.488/1000)*9.81;
	buffer_xyz[1] = ((double)y_val * 0.488/1000)*9.81;
	buffer_xyz[2] = ((double)z_val * 0.488/1000)*9.81;
}
