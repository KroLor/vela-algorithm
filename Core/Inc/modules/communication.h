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
	SYS_AREA_NONE,
	SYS_AREA_INIT,
	SYS_AREA_MAIN_ALGO,
	SYS_AREA_READ_SENSORS,
	SYS_AREA_PERIPH_SDCARD,
	SYS_AREA_PERIPH_RADIO,
	SYS_AREA_PERIPH_BAROM,
	SYS_AREA_PERIPH_ACC

} SystemArea;

typedef enum {
	STORAGE_RADIO = 1,
    STORAGE_SD = 1 << 1,
} enabled_storage;

typedef struct {
	SystemState sys_state;
	SystemArea sys_area;
	Msg_Priority priority;
	char *text;
} Message;

/// @brief Конвертирует статус регистра в читабельное сообщение и отправляет его
/// @param status Статус регистра
/// @param reg Название регистра
void log_register(HAL_StatusTypeDef status, char *reg, SystemState sys_state, SystemArea sys_area);

/// @brief Отправляет данные в сконфигурированные хранилища информации (память/радио/юарт/т.п.)
/// @param msg Текстовое сообщение
/// @param priority Приоритет сообщения
void send_message(char *msg, Msg_Priority priority);

/// @brief Отправляет данные в сконфигурированные хранилища информации (память/радио/юарт/т.п.)
void log_message(Message* msg);

void send_status(uint8_t status);
