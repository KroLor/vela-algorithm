#include "servo.h"
#include "system_definitions.h"

/*
static const uint16_t max_pulse = 25; // 2.5 милисекунды
static const uint16_t min_pulse = 5;  // 0.5 милисекунды
static uint16_t curr_pulse = 19;
*/

static const uint16_t max_pulse = 18; // 2 милисекунды
static const uint16_t min_pulse = 3;  // 1 милисекунды
static const uint16_t apogy_pulse = 10;
static uint16_t curr_pulse = 19;

void set_pulse(uint16_t pulse)
{
	curr_pulse = pulse;
	SERVO_TIM_DEF->CCR2 = pulse;
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 19);
}

short servo_turn_percentage(uint8_t percentage)
{
	uint16_t full_range = max_pulse - min_pulse;
	uint16_t new_val = full_range * ((double)percentage) / 100;
	new_val += 5;

	set_pulse(new_val);

	return 0;
}

short servo_turn_apogy()
{
	set_pulse(apogy_pulse);
	return 0;
}

short servo_turn_max()
{
	set_pulse(max_pulse);
	return 0;
}

short servo_turn_min()
{
	set_pulse(min_pulse);
	return 0;
}