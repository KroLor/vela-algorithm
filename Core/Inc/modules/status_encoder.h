#include <stdint.h>

// Current mapping:
// ┌───────────────┬───────┬───────┬─────────┬───────────────┬───────────┬─────┐
// │ bits ( = LED) │ 1     │ 2     │ 3       │ 4             │ 5         │ 6   │
// ├───────────────┼───────┼───────┼─────────┼───────────────┼───────────┼─────┤
// │ sensor        │ servo │ radio │ sd card │ accelerometer │ barometer │ gps │
// └───────────────┴───────┴───────┴─────────┴───────────────┴───────────┴─────┘

inline void status_barometer_responds(uint8_t status);
inline void status_acc_responds(uint8_t status);
inline void status_radio_responds(uint8_t status);
inline void status_servo_responds(uint8_t status);
inline void status_sd_mounts(uint8_t status);
inline void status_gps_responds(uint8_t status);
