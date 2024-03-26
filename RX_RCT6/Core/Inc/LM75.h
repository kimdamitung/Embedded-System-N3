/*
 * LM75.h
 *
 *  Created on: Mar 26, 2024
 *      Author: duytung
 */

#ifndef INC_LM75_H_
#define INC_LM75_H_

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "main.h"

extern uint8_t Sign;

#define LM75AD_devAddr 0x48
#define LM75AD_TempReg 0x00

float LM75A_GetTemperature(void);

#endif /* INC_LM75_H_ */
