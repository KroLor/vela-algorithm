#include "accelerometer.h"
#include <stdint.h>
#include "i2c.h"
#include "usart.h"

short check_acc_identity(uint8_t* identity_reg_buffer, void (*f)(HAL_StatusTypeDef, char *))
{
	uint8_t dev_address = 0b11010100; //адрес устройства по линии I2C
	uint16_t register_address = 0x0F; //Адрес регистра в котором хранится значение ID
	uint8_t data; //Массив в котором МЫ будем хранить данные с регистра устройства
	uint16_t Size_ = 1; //Длина запрашиваемых данных, 1 байт = 1 регистр
	uint32_t Timeout_ = 0xFF; //Таймаут, 255 мс

	(*f)(HAL_I2C_Mem_Read(&hi2c1, dev_address, register_address, I2C_MEMADD_SIZE_8BIT, &data, Size_, Timeout_), "WHO AM I");
	if (data == 0x69)
	{
        //successfuly read register
        return 1;

        uint8_t acc_power_mode = 0b01000100;
        UART_Send_HAL_Status(HAL_I2C_Mem_Write(&hi2c1, dev_address, 0x10, I2C_MEMADD_SIZE_8BIT, &acc_power_mode, 1, 0xFF), "ctrl_meas");
	} else
	{
		char buffer [50] = "ACCELEROMETER READ ERROR\n\r";
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 1000);
    }
}