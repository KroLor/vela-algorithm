#include <stdint.h>

/// @brief поворачивает серво на определенный процент.
/// @param percentage процент, отсчитываемый от минимального положения
short servo_turn_percentage(uint8_t percentage);

/// @brief поворачивает серво на максимальное значение
short servo_turn_max();

/// @brief поворачивает серво на минимальное значение
/// @return 
short servo_turn_min();

short servo_turn_apogy();