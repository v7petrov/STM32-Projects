/*
 * keypad.h
 *
 *  Created on: Oct 26, 2024
 *      Author: USER
 */

#include "stm32l4xx_hal.h"

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#define NUM_ROWS 4
#define NUM_COLS 4
#define ROW_PORT (GPIO_IDR_ID0 | GPIO_IDR_ID1 | GPIO_IDR_ID2 | GPIO_IDR_ID3)
#define COL_PORT (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD7)

void keypad_init(void);
uint8_t getKeypadResult(uint8_t);
int8_t getKeypadNumber(void);

#endif /* INC_KEYPAD_H_ */
