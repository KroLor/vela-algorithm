#include "usart.h"
#include "radio.h"
#include "system_definitions.h"

bool is_enabled = false;
static const uint32_t uart_timeout_default = 0xFF; // Таймаут, 255 мс
static const uint16_t module_addr = 0x69;

//Actual frequency = 410.125 + CH *1M
static const uint8_t channel = 23;

bool radio_is_enabled()
{
	return is_enabled;
}

void radio_init()
{
	//set m0,m1 high - go to sleep, start configuration
	HAL_GPIO_WritePin(RADIO_M0_PORT, RADIO_M0_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RADIO_M1_PORT, RADIO_M1_PIN, GPIO_PIN_SET);
	HAL_Delay(50);  // Critical wait for mode transition

	//Configuration goes here

	uint8_t msg_cmd[4] = { 0xc0, 0x00, 1, module_addr >> 8};
	/*
	//Write ADDH - high bytes of address of module
	HAL_UART_Transmit(&RADIO_UART_HANDLE, msg_cmd, 4, uart_timeout_default);
	HAL_Delay(10);

	//Write ADDL - low bytes of address of module
	msg_cmd[1] = 0x01; //register addr
	msg_cmd[3] = module_addr & 255; //register data
	HAL_UART_Transmit(&RADIO_UART_HANDLE, msg_cmd, 4, uart_timeout_default);
	HAL_Delay(10);
	*/

	//Write ADDL - low bytes of address of module
	msg_cmd[1] = 0x05; //register addr
	msg_cmd[3] = channel; //register data
	HAL_UART_Transmit(&RADIO_UART_HANDLE, msg_cmd, 4, uart_timeout_default);
	HAL_Delay(10);

	//set m0,m1 low - go to normal mode, open both uart channels
	HAL_GPIO_WritePin(RADIO_M0_PORT, RADIO_M0_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RADIO_M1_PORT, RADIO_M1_PIN, GPIO_PIN_RESET);
	HAL_Delay(50);  // Wait for mode transition

	is_enabled = true;
}
