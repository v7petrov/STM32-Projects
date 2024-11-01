/*
 * DAC.h
 *
 *  Created on: Oct 26, 2024
 *      Author: USER
 */
#include "stm32l4xx_hal.h"

#ifndef INC_DAC_H_
#define INC_DAC_H_

#define DAC_MAX 4095
#define VOLT_MAX 3300
#define DAC_MASK 0x3000

#define WEIGHTS 0.993

void DAC_init(void);
void DAC_write(uint16_t);
uint16_t DAC_volt_conv (uint16_t);

#endif /* INC_DAC_H_ */
