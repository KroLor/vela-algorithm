#include "communication.h"
#include "system_definitions.h"
#include "sd_card.h"
#include "radio.h"
#include <string.h>
#include <usart.h>
#include <stdio.h>

static const uint32_t timeout_default = 0xFF; // Таймаут, 255 мс
static char log_file_name[] = "/sys.log";

void log_register(HAL_StatusTypeDef status, char *reg, SystemState sys_state, SystemArea sys_area)
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
			message = "UNKNOWN";
			break;
	}

	char buffer[256];
	sprintf(buffer, "%s: %s\r\n", reg, message);

	Message res_msg = { .text = buffer, .sys_state = sys_state, .sys_area = sys_area, .priority = PRIORITY_LOW };
	log_message(&res_msg);
}

void system_state_to_str(char* msg, SystemState state)
{
	switch (state)
	{
		case SYS_STATE_NONE:
			sprintf(msg, "none");
			break;
		case SYS_STATE_INIT:
			sprintf(msg, "init");
			break;
		case SYS_STATE_STANDBY:
			sprintf(msg, "standby");
			break;
		case SYS_STATE_LIFTOFF:
			sprintf(msg, "liftoff");
			break;
		case SYS_STATE_ASCENT:
			sprintf(msg, "ascent");
			break;
		case SYS_STATE_APOGY:
			sprintf(msg, "apogy");
			break;
		case SYS_STATE_DESCENT:
			sprintf(msg, "descent");
			break;
		case SYS_STATE_GROUND:
			sprintf(msg, "ground");
			break;
		default:
			sprintf(msg, "err");
			break;
	}
}

void system_area_to_str(char* msg, SystemArea area)
{
	switch (area)
	{
		case SYS_AREA_NONE:
			sprintf(msg, "none");
			break;
		case SYS_AREA_INIT:
			sprintf(msg, "init");
			break;
		case SYS_AREA_MAIN_ALGO:
			sprintf(msg, "main_algo");
			break;
		case SYS_AREA_READ_SENSORS:
			sprintf(msg, "read_sensors");
			break;
		case SYS_AREA_PERIPH_SDCARD:
			sprintf(msg, "periph_sdcard");
			break;
		case SYS_AREA_PERIPH_RADIO:
			sprintf(msg, "periph_radio");
			break;
		case SYS_AREA_PERIPH_BAROM:
			sprintf(msg, "periph_barom");
			break;
		case SYS_AREA_PERIPH_ACC:
			sprintf(msg, "periph_acc");
			break;
		default:
			sprintf(msg, "err");
			break;
	}
}

void send_message(char *msg, Msg_Priority priority)
{
	Message msg_struct = { .sys_area = SYS_AREA_NONE, .sys_state = SYS_STATE_NONE, .priority = priority, .text = msg };
	log_message(&msg_struct);
}

void log_message(Message* msg)
{
	char state_text[64];
	system_state_to_str(state_text, msg->sys_state);

	char area_text[64];
	system_area_to_str(area_text, msg->sys_area);

	uint16_t log_size = strlen(msg->text) + strlen(state_text) + strlen(area_text) + 3;
	char* log_text = malloc(log_size);

	uint32_t curr_ms = HAL_GetTick();

	//state;area;text
	sprintf(log_text, "%u;%s;%s;%s", curr_ms, state_text, area_text, msg->text);

	if (sd_card_is_enabled())
	{
		sd_file file;
		sd_status sd_stat = sd_card_open_file(&file, log_file_name);

		if (sd_stat == SD_OK)
		{
			sd_stat = sd_card_write(&file, log_text);
			sd_card_close(&file);
		}
	}

	if(radio_is_enabled())
	{
		HAL_UART_Transmit(&RADIO_UART_HANDLE, (uint8_t *)log_text, log_size, timeout_default);
	}

	free(log_text);
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
