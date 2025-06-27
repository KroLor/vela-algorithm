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

static SystemState curr_sys_state = SYS_STATE_NONE;

SystemState get_sys_state()
{
	return curr_sys_state;
}

void read_sensors()
{
	Message msg = { .sys_area = SYS_AREA_READ_SENSORS, .sys_state = curr_sys_state, .priority = PRIORITY_HIGH };
	msg.text = malloc(256);

	msg.priority = PRIORITY_DEBUG;
	sprintf(msg.text, "[begin reading sensors]_____________\n\r");
	log_message(&msg);

	//BAROMETER
	if (sensors_status & SENSOR_BAROM)
	{
		msg.priority = PRIORITY_DEBUG;
		sprintf(msg.text, "[reading barometer]_____\n\r");
		log_message(&msg);

		int32_t actual_temp = read_temp();

		msg.priority = PRIORITY_HIGH;
		sprintf(msg.text, "Temperature: %.2f Celsius\n\r", ((float)actual_temp) / 100);
		log_message(&msg);

		uint32_t actual_pressure = read_pressure();

		sprintf(msg.text, "Pressure: %.4f Pa\n\r", ((float)actual_pressure) / 256);
		log_message(&msg);

		float actual_height = get_height()/* - start_height*/;

		sprintf(msg.text, "Height: %.4f m\n\r", ((float)actual_height));
		log_message(&msg);
	}
	else
	{
		msg.priority = PRIORITY_DEBUG;
		sprintf(msg.text, "barometer disabled!\n\r");
		log_message(&msg);
	}
	
	//ACCELEROMETER
	if (sensors_status & SENSOR_ACC)
	{
		msg.priority = PRIORITY_DEBUG;
		sprintf(msg.text, "[reading accelerometer]_____\n\r");
		log_message(&msg);

		double acc_vals[3];
		read_acceleration_xyz(acc_vals);

		msg.priority = PRIORITY_HIGH;
		sprintf(msg.text, "Acceleration: (%0.4f, %0.4f, %0.4f) \n\r", acc_vals[0], acc_vals[1], acc_vals[2]);
		log_message(&msg);
	}
	else
	{
		msg.priority = PRIORITY_DEBUG;
		sprintf(msg.text, "accelerometer disabled!!\n\r");
		log_message(&msg);
	}

	msg.priority = PRIORITY_DEBUG;
	sprintf(msg.text, "[end reading sensors]_____________\n\r");
	log_message(&msg);

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

bool check_rescue() {
	// Проверяем концевую кнопку
	if (HAL_GPIO_ReadPin(END_BUTTON_PORT, END_BUTTON_PIN) == GPIO_PIN_SET) {
		return 1;
	}

	// Проверяем фоторезистор
	uint32_t currentValue = HAL_ADC_GetValue(&hadc1);
	int32_t difference = currentValue - previousValue;

	if (difference > threshold) {
	  // Резкое осветление
	  return 1;
	}
	previousValue = currentValue;

	return 0;
}

bool check_apogy() {
	// Проверяем высоту (get_height())
	

	// Проверяем акселерометр


	return 0;
}

void open_rescue() {
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

void apogy()
{
	curr_sys_state = SYS_STATE_APOGY;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	Message msg = { .sys_area = SYS_AREA_MAIN_ALGO, .sys_state = SYS_STATE_APOGY, .priority = PRIORITY_HIGH };
	msg.text = malloc(256);
	
	sprintf(msg.text, "Apogy! Initiating rescue.\r\n");
	log_message(&msg);

	//Just don't think, fire it multiple times
	for (size_t i = 0; i < 3; i++)
	{
		open_rescue();

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}

	//Hope that it works
	if (check_rescue())
	{
		sprintf(msg.text, "Rescue system open!\r\n");
		log_message(&msg);
	}
	else
	{
		sprintf(msg.text, "Rescue system failed to open! Retrying...\r\n");
		log_message(&msg);

		//Try a bit more
		for (size_t i = 0; i < 5; i++)
		{
			open_rescue();

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_Delay(300);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		}

		if (check_rescue())
		{
			sprintf(msg.text, "Rescue system finally opened!\r\n");
			log_message(&msg);
		}
		else
		{
			sprintf(msg.text, "Rescue system failed to open anyway.\r\n");
			log_message(&msg);
		}
	}

	free(msg.text);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void start_flight()
{
	curr_sys_state = SYS_STATE_LIFTOFF;
	send_status(0x0);

	Message msg = { .text = "🚀 Поплыли к звездам! 🚀 \n\n\n\r\0", .sys_area = SYS_AREA_MAIN_ALGO, .sys_state = curr_sys_state, .priority = PRIORITY_HIGH };
	log_message(&msg);

	//start reading sensors
	__HAL_TIM_SET_COUNTER(&SENSORS_READ_TIM_HANDLE, 0);
	__HAL_TIM_CLEAR_FLAG(&SENSORS_READ_TIM_HANDLE, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&SENSORS_READ_TIM_HANDLE);

	//count down ot apogy
	__HAL_TIM_SET_COUNTER(&APOGY_TIM_HANDLE, 0);
	__HAL_TIM_CLEAR_FLAG(&APOGY_TIM_HANDLE, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&APOGY_TIM_HANDLE);
}

void initialize_system()
{
	curr_sys_state = SYS_STATE_INIT;

	// Крутим вентилятор
	HAL_GPIO_WritePin(vent_GPIO_Port, vent_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(vent_GPIO_Port, vent_Pin, GPIO_PIN_RESET);

	Message msg = { .sys_area = SYS_AREA_INIT, .sys_state = SYS_STATE_INIT, .priority = PRIORITY_HIGH };
	msg.text = malloc(256);
	uint8_t status = 0x0;

	sprintf(msg.text, "_____________ [begin system init] \n\r");
	log_message(&msg);

	//0. SD CARD - the first, to enable log to it right away.
	msg.sys_area = SYS_AREA_PERIPH_SDCARD;
	sprintf(msg.text, "_____[init: sd card]\n\r");
	log_message(&msg);

	sd_status sd_stat = sd_card_mount();

	if (sd_stat == SD_OK)
	{
		sprintf(msg.text, "sd card mounted\r\n");
		log_message(&msg);

		sd_file file;
		sd_stat = sd_card_open_file(&file, "/test");

		if (sd_stat == SD_OK)
		{
			sprintf(msg.text, "sd card test file opened\r\n");
			log_message(&msg);

			sd_stat = sd_card_write(&file, "Good luck, good flight.\r\n");

			if (sd_stat == SD_OK)
			{
				sprintf(msg.text, "sd card test file written\r\n");
				log_message(&msg);
			}
			else
			{
				sprintf(msg.text, "!!sd card test file write failure!!\r\n");
				log_message(&msg);
			}

			sd_card_close(&file);
		}
		else
		{
			sprintf(msg.text, "!!sd card test file failed to open!!\r\n");
			log_message(&msg);
		}

		_sd_card_set_enabled();
		status_sd_mounts(&status);
	}
	else
	{
		sprintf(msg.text, "!!!sd card failed to mount!!!\r\n");
		log_message(&msg);
	}

	//1. Radio
	msg.sys_area = SYS_AREA_PERIPH_RADIO;
	sprintf(msg.text, "[init: radio]_____\n\r");
	log_message(&msg);

	HAL_Delay(1000);
	radio_init();
	status_radio_responds(&status);

	//2. ACCELEROMETER
	msg.sys_area = SYS_AREA_PERIPH_ACC;
	sprintf(msg.text, "[init: acc]_____\r\n");
	log_message(&msg);

	if (check_acc_identity())
	{
		sprintf(msg.text, "accelerometer responds nicely, powering it on...\r\n");
		log_message(&msg);

		acc_power_on();
		sensors_status |= SENSOR_ACC;

		status_acc_responds(&status);
	}
	else
	{
		sprintf(msg.text, "!!!accelerometer not responding!!!\n\r");
		log_message(&msg);
	}

	//3. BAROMETER
	msg.sys_area = SYS_AREA_PERIPH_BAROM;
	sprintf(msg.text, "[init: barometer]_____\n\r");
	log_message(&msg);

	if (check_barometer_identity())
	{
		sprintf(msg.text, "barometer responds correctly, powering on...\n\r");
		log_message(&msg);

		sensors_status |= SENSOR_BAROM;

		barometer_power_on();

		status_barometer_responds(&status);
	}
	else
	{
		sprintf(msg.text, "!!!barometer not responding!!!\n\r");
		log_message(&msg);
	}

	//4. SERVO
	HAL_TIM_PWM_Start(&SERVO_TIM_HANDLE, SERVO_TIM_PWM_CHANNEL);

	status_servo_responds(&status);

	//servo_turn_min();

	send_status(status);

	msg.sys_state = SYS_AREA_INIT;
	sprintf(msg.text, "[end system init]_____________\n\r");
	log_message(&msg);

	free(msg.text);
}
