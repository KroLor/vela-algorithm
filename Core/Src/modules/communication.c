#include "communication.h"
#include <string.h>
#include <usart.h>

void send_reg_log(HAL_StatusTypeDef status, char *reg, Msg_Priority priority)
{
  char* message = NULL;

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

  send_message(buffer, priority);
}

void send_message(char *msg, Msg_Priority priority)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 0xFF);
}
