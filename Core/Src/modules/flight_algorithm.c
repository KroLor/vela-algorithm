#include "flight_algorithm.h"
#include "system_definitions.h"
#include "status_encoder.h"
#include "communication.h"
#include "barometer.h"
#include "accelerometer.h"
#include "tim.h"
#include "sd_card.h"
#include "radio.h"
#include "servo.h"
#include "adc.h"
#include <stdio.h>
#include <math.h>

#define SEA_LEVEL_PRESSURE 101325.0f // Стандартное давление на уровне моря (Па)
#define GRAVITY 9.81f // Ускорение свободного падения (м/с²)
#define GAS_CONSTANT 287.0f // Удельная газовая постоянная для воздуха (Дж/(кг·K))
#define LAPSE_RATE 0.0065f // Градиент температуры (K/m)

// float start_height = 0.0f;
uint32_t previousValue = 0;
const uint32_t threshold = 200;  // Порог изменения (нужно подбирать)

static uint8_t sensors_status = 0;
static bool is_liftoff = false;

void read_sensors()
{
	char msg[256];
	sprintf(msg, "_____________[begin reading sensors]_____________\n\r");
	send_message(msg, PRIORITY_DEBUG);

	//BAROMETER
	if (sensors_status & SENSOR_BAROM)
	{
		sprintf(msg, "_____[reading barometer]_____\n\r");
		send_message(msg, PRIORITY_DEBUG);

		int32_t actual_temp = read_temp();

		char temp_str[100];
		sprintf(temp_str, "Temperature: %.2f Celsius\n\n\r", ((float)actual_temp) / 100);
		send_message(temp_str, PRIORITY_HIGH);

		uint32_t actual_pressure = read_pressure();

		char pressure_str[100];
		sprintf(pressure_str, "Pressure: %.4f Pa\n\n\r", ((float)actual_pressure) / 256);
		send_message(pressure_str, PRIORITY_HIGH);

		float actual_height = get_height()/* - start_height*/;

		char height_str[100];
		sprintf(height_str, "Height: %.4f m\n\n\r", ((float)actual_height));
		send_message(height_str, PRIORITY_HIGH);
	}
	else
	{
		sprintf(msg, "!!barometer disabled!!\n\r");
		send_message(msg, PRIORITY_DEBUG);
	}
	
	//ACCELEROMETER
	if (sensors_status & SENSOR_ACC)
	{
		sprintf(msg, "_____[reading accelerometer]_____\n\r");
		send_message(msg, PRIORITY_DEBUG);

		double acc_vals[3];
		read_acceleration_xyz(acc_vals);

		char acc_str[100];
		sprintf(acc_str, "Acceleration: (%0.4f, %0.4f, %0.4f) \n\n\r", acc_vals[0], acc_vals[1], acc_vals[2]);
		send_message(acc_str, PRIORITY_HIGH);
	}
	else
	{
		sprintf(msg, "!!accelerometer disabled!!\n\r");
		send_message(msg, PRIORITY_DEBUG);
	}

	sprintf(msg, "_____________[end reading sensors]_____________\n\r");
	send_message(msg, PRIORITY_DEBUG);

/*
//Servo stuff
		if (pwm_switch)
		{
			servo_turn_min();
			pwm_switch = 0;
		}
		else
		{
			servo_turn_max();
			pwm_switch = 1;
		}
*/
}

bool check_res_sys(char* count_check_apogee) {
	// Проверяем концевую кнопку
	if (HAL_GPIO_ReadPin(END_BUTTON_PORT, END_BUTTON_PIN) == GPIO_PIN_SET) {
		return 0;
	}

	// Проверяем фоторезистор
	uint32_t currentValue = HAL_ADC_GetValue(&hadc1);
    int32_t difference = currentValue - previousValue;

    if (difference > threshold) {
      // Резкое осветление
      return 0;
    }
    previousValue = currentValue;

	(*count_check_apogee)++;
	return 1;
}

bool check_apogy() {
	// Проверяем высоту (get_height())
	

	// Проверяем акселерометр


	return 0;
}

void res_sys() {
	servo_turn_apogy();
	HAL_Delay(1000); //
	servo_turn_max();
}

bool check_landing() {
	// Проверяем высоту (get_height())


	// Проверяем акселерометр
	

	return 0;
}

// void get_start_height() {
// 	if (sensors_status & SENSOR_BAROM) {
// 		start_height = get_height();
// 	}
// }

float get_height()
{
    float pressure = (float)read_pressure() / 256.0f;
    float temperature = (float)read_temp() / 100.0f;
    
    if(pressure <= 0 || pressure > SEA_LEVEL_PRESSURE * 1.5f) {
        return -9999.0f;  // Некорректное давление
    }
    
    float temp_kelvin = temperature + 273.15f;
    
    float height = (temp_kelvin / LAPSE_RATE) * 
                  (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 
                  (GAS_CONSTANT * LAPSE_RATE) / GRAVITY));
    
    return height;
}

void start_flight()
{
	if (!is_liftoff)
	{
		char touched_off_msg[] = "\n\n\n\r🚀 Поплыли к звездам! 🚀 \n\n\n\r\0";
		send_message(touched_off_msg, PRIORITY_HIGH);
		is_liftoff = true;

		//start sensors reading timer
		HAL_TIM_Base_Start_IT(&SENSORS_READ_TIM_HANDLE);
		// HAL_TIM_Base_Start_IT(&APOGY_TIM_HANDLE); //
	}
}

void initialize_system()
{
	uint8_t status = 0x0;
	char msg[256];
	sprintf(msg, "_____________ [begin system init] _____________\n\r");
	send_message(msg, PRIORITY_HIGH);
/*
	//1. Radio
	sprintf(msg, "_____[init: radio]_____\n\r");
	send_message(msg, PRIORITY_HIGH);

	radio_init();
	status_radio_responds(&status);
*/

	//1. SD CARD - the first, to enable log to it right away.
	sprintf(msg, "_____[init: sd card]_____\n\r");
	send_message(msg, PRIORITY_HIGH);

	sd_status sd_stat = sd_card_mount();

	if (sd_stat == SD_OK)
	{
		sprintf(msg, "sd card mounted\r\n");
		send_message(msg, PRIORITY_HIGH);

		sd_file file;
		sd_stat = sd_card_open_file(&file, "/test");

		if (sd_stat == SD_OK)
		{
			sprintf(msg, "sd card test file opened\r\n");
			send_message(msg, PRIORITY_HIGH);

			sd_stat = sd_card_write(&file, "Good luck, good flight.\r\n");

			if (sd_stat == SD_OK)
			{
				sprintf(msg, "sd card test file written\r\n");
				send_message(msg, PRIORITY_HIGH);
			}
			else
			{
				sprintf(msg, "!!sd card test file write failure!!\r\n");
				send_message(msg, PRIORITY_HIGH);
			}

			sd_card_close(&file);
		}
		else
		{
			sprintf(msg, "!!sd card test file failed to open!!\r\n");
			send_message(msg, PRIORITY_HIGH);
		}

		_sd_card_set_enabled();
		status_sd_mounts(&status);
	}
	else
	{
		sprintf(msg, "!!!sd card failed to mount!!!\r\n");
		send_message(msg, PRIORITY_HIGH);
	}

	//2. ACCELEROMETER
	sprintf(msg, "_____[init: acc]_____\r\n");
	send_message(msg, PRIORITY_HIGH);

	if (check_acc_identity())
	{
		sprintf(msg, "accelerometer responds nicely, powering it on...\r\n");
		send_message(msg, PRIORITY_HIGH);

		acc_power_on();
		sensors_status |= SENSOR_ACC;

		status_acc_responds(&status);
	}
	else
	{
		sprintf(msg, "!!!accelerometer not responding!!!\n\r");
		send_message(msg, PRIORITY_HIGH);
	}

	//3. BAROMETER
	sprintf(msg, "_____[init: barometer]_____\n\r");
	send_message(msg, PRIORITY_HIGH);
	if (check_barometer_identity())
	{
		sprintf(msg, "barometer responds correctly, powering on...\n\r");
		send_message(msg, PRIORITY_HIGH);
		sensors_status |= SENSOR_BAROM;

		barometer_power_on();

		status_barometer_responds(&status);
	}
	else
	{
		sprintf(msg, "!!!barometer not responding!!!\n\r");
		send_message(msg, PRIORITY_HIGH);
	}

	//4. SERVO
	HAL_TIM_PWM_Start(&SERVO_TIM_HANDLE, SERVO_TIM_PWM_CHANNEL);

	status_servo_responds(&status);

	//servo_turn_min();

	send_status(status);

	sprintf(msg, "_____________[end system init]_____________\n\r");
	send_message(msg, PRIORITY_HIGH);
}
