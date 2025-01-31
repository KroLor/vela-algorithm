#include "main.h"

typedef enum {
  LOW,
  MEDIUM,
  HIGH
} Msg_Priority; 

void send_reg_log(HAL_StatusTypeDef status, char* reg, Msg_Priority priority);
void send_message(char* msg, Msg_Priority priority);
