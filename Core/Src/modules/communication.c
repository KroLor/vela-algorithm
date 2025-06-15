#include "communication.h"
#include "system_definitions.h"
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
	HAL_UART_Transmit(&USB_UART_HANDLE, (uint8_t *)msg, strlen(msg), timeout_default);
	
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
	// else if (radio_is_enabled()) {
	// 	radio_send_message(msg);
	// }
}

void send_status(uint8_t status)
{
	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, status & 1);
	HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, status & (1 << 1));
	HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, status & (1 << 2));
	HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, status & (1 << 3));
	HAL_GPIO_WritePin(LED5_PORT, LED5_PIN, status & (1 << 4));
	HAL_GPIO_WritePin(LED6_PORT, LED6_PIN, status & (1 << 5));
}
