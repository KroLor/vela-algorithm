#include "barometer.h"
#include "communication.h"
#include <i2c.h>

short check_barometer_identity()
{
	uint16_t Address_Bmp280 = 0x76 << 1; //или 0x77 - адрес устройства по линии I2C
	uint16_t Register_ID = 0xD0; //Адрес регистра в котором хранится значение ID
	uint8_t Data[1]; //Массив в котором МЫ будем хранить данные с регистра устройства
	uint16_t Size_ = 1; //Длина запрашиваемых данных, 1 байт = 1 регистр
	uint32_t Timeout_ = 0xFF; //Таймаут, 255 мс

	HAL_I2C_Mem_Read(&hi2c1, Address_Bmp280, Register_ID, I2C_MEMADD_SIZE_8BIT, Data, Size_, Timeout_);
	if (Data[0] == 0x58)
	{
        return 1;
	} else
	{
        return 0;
	}

    return 0;
}