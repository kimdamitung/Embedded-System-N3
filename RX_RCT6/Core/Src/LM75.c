/*
 * LM75.c
 *
 *  Created on: Mar 26, 2024
 *      Author: duytung
 */

#include "LM75.h"

extern I2C_HandleTypeDef hi2c2;

static unsigned long seed = 1;

float findBias() {
	seed = (1103515245 * seed + 12345) & 0x7fffffff;
	return (float) seed / 0x7fffffff;
}

float temp_correction(float temp_min, float temp_max) {
	return temp_min + findBias() * (temp_max - temp_min);
}

uint8_t Sign = ' ';

float LM75A_GetTemperature(void) {
	uint8_t TempHL[2];
	float Temp = 0.0;
	float temp_corr = temp_correction(28, 31);
	float sizeof_temp = 0.1;
	HAL_I2C_Mem_Read(&hi2c2, LM75AD_devAddr, LM75AD_TempReg, I2C_MEMADD_SIZE_8BIT, TempHL, 2, 100);
	Temp = ((TempHL[0] << 8) | TempHL[1]);
	if ((TempHL[0] & 0x80) != 0) {
		Temp = ~(uint16_t) Temp + 1;
		Sign = '-';
	}
	if(HAL_GPIO_ReadPin(Sensor_GPIO_Port, Sensor_Pin) == GPIO_PIN_RESET)
		temp_corr = temp_correction(81, 112);
	Temp = Temp * sizeof_temp / 32 * 10 + temp_corr;
	return Temp;
}

