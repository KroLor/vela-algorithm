#include "main.h"

typedef enum {
  PRIORITY_LOW,
  PRIORITY_MEDIUM,
  PRIORITY_HIGH
} Msg_Priority; 

void send_reg_log(HAL_StatusTypeDef status, char* reg);
void send_message(char* msg, Msg_Priority priority);
