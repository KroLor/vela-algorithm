#include "communication.h"
#include "sd_card.h"
#include <string.h>
#include <usart.h>

static const uint32_t timeout_default = 0xFF; // Таймаут, 255 мс
static char log_file_name[] = "/sys.log";

void send_reg_log(HAL_StatusTypeDef status, char *reg)
{
	char *message = NULL;

	switch (status)
	{
	case HAL_OK:
		message = "OK";
		break;
	case HAL_ERROR:
		message = "ERROR";
		break;

	case HAL_BUSY:
		message = "BUSY";
		break;
	case HAL_TIMEOUT:
		message = "TIMEOUT";
		break;

	default:
		break;
	}

	char buffer[100];
	strcat(buffer, reg);
	strcat(buffer, ": ");
	strcat(buffer, message);
	strcat(buffer, "\n\r\0");

	send_message(buffer, PRIORITY_LOW);
}

void send_message(char *msg, Msg_Priority priority)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), timeout_default);

	if (sd_card_is_enabled())
	{
		sd_file file;
		sd_status sd_stat = sd_card_open_file(&file, log_file_name);

		if (sd_stat == SD_OK)
		{
			sd_stat = sd_card_write(&file, msg);
			sd_card_close(&file);
		}
	}
}
