#include "main.h"

typedef enum
{
	PRIORITY_LOW,
	PRIORITY_MEDIUM,
	PRIORITY_HIGH
} Msg_Priority;

/// @brief Конвертирует статус регистра в читабельное сообщение и отправляет его
/// @param status Статус регистра
/// @param reg Название регистра
void send_reg_log(HAL_StatusTypeDef status, char *reg);

/// @brief Отправляет данные в сконфигурированные хранилища информации (память/радио/юарт/т.п.)
/// @param msg Текстовое сообщение
/// @param priority Приоритет сообщения
void send_message(char *msg, Msg_Priority priority);
