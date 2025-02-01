#include <stdint.h>

/// @brief Делает тестовый запрос к устройству
/// @returns Возвращает 1 если ответило правильное устройство, 0 иначе
short check_barometer_identity();

/// @brief Устанавливает режим работы в устройство
short barometer_power_on();

/// @brief Считывает и возвращает значение давления
uint32_t read_pressure();

/// @brief Считывает и возвращает значение температуры 
int32_t read_temp();