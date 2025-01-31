#include "accelerometer.h"
#include <stdint.h>
#include "i2c.h"
#include "usart.h"
#include "communication.h"
#include "string.h"

const uint8_t dev_address = 0b11010100; //адрес устройства по линии I2C

short check_acc_identity()
{
	uint16_t register_address = 0x0F; //Адрес регистра в котором хранится значение ID
	uint8_t data; //Массив в котором МЫ будем хранить данные с регистра устройства
	uint16_t Size_ = 1; //Длина запрашиваемых данных, 1 байт = 1 регистр
	uint32_t Timeout_ = 0xFF; //Таймаут, 255 мс

	send_reg_log(HAL_I2C_Mem_Read(&hi2c1, dev_address, register_address, I2C_MEMADD_SIZE_8BIT, &data, Size_, Timeout_), "WHO AM I");
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
	send_reg_log(HAL_I2C_Mem_Write(&hi2c1, dev_address, 0x10, I2C_MEMADD_SIZE_8BIT, &acc_power_mode, 1, 0xFF), "ctrl_meas");

    return 0;
}
