#include "main.h"

typedef enum
{
	PRIORITY_DEBUG,
	PRIORITY_LOW,
	PRIORITY_MEDIUM,
	PRIORITY_HIGH
} Msg_Priority;

typedef enum
{
	StateChange,
	SdCard

} MsgArea;

typedef enum {
	STORAGE_RADIO = 1,
    STORAGE_SD = 1 << 1,
} enabled_storage;

/// @brief Конвертирует статус регистра в читабельное сообщение и отправляет его
/// @param status Статус регистра
/// @param reg Название регистра
void send_reg_log(HAL_StatusTypeDef status, char *reg);

/// @brief Отправляет данные в сконфигурированные хранилища информации (память/радио/юарт/т.п.)
/// @param msg Текстовое сообщение
/// @param priority Приоритет сообщения
void send_message(char *msg, Msg_Priority priority);

