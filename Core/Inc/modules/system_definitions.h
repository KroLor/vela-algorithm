#include <stm32f411xe.h>
#include <time.h>

#define SENSORS_READ_TIM htim2
#define SERVO_TIM_DEF TIM1
#define SERVO_TIM_HANDLE htim1
#define SD_SPI_HANDLE hspi1
#define JUMPER_PIN GPIO_PIN_1
