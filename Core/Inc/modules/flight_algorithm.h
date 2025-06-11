#include <stdbool.h>

typedef enum {
    SENSOR_BAROM = 1,
    SENSOR_ACC = 1 << 1
} enabled_sensors;

void initialize_system();

void start_flight();

void read_sensors();

// Активирует систему спасения
void res_sys();
// Вернёт 0, если система спасения отработала штатно
bool check_res_sys(char* count_check_apogee);
// Вернёт 1, если апогей
bool check_apogy();
// Вернёт 1, если приземление
bool check_landing();
// Возвращает текущую высоту
// uint32_t get_height();