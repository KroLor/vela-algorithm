#include "status_encoder.h"

void status_servo_responds(uint8_t status)
{
	status |= 1;
}

void status_radio_responds(uint8_t status)
{
	status |= 1 << 1;
}

void status_sd_mounts(uint8_t status)
{
	status |= 1 << 2;
}

void status_acc_responds(uint8_t status)
{
	status |= 1 << 3;
}

void status_barometer_responds(uint8_t status)
{
	status |= 1 << 4;
}

void status_gps_responds(uint8_t status)
{
	status |= 1 << 5;
}
