#include <stdbool.h>
#include "ff.h"

typedef struct {
    uint32_t total_space;
    uint32_t free_space;
} sd_card_info;

typedef enum {
    SD_OK = 0,
    SD_ERR = 1
} sd_status;

typedef struct {
    FIL internal_handle;
} sd_file;

bool sd_card_is_enabled();

void _sd_card_set_enabled();

/// @brief Устанавливает режим работы в устройство
sd_status sd_card_mount();

sd_status sd_card_get_info(sd_card_info* info);

sd_status sd_card_open_file(sd_file* file_handle, char* filepath);

sd_status sd_card_write(sd_file* file_handle, char* str);

sd_status sd_card_close(sd_file* file_handle);
