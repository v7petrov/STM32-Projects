/*
 * keypad.c
 *
 *  Created on: Oct 26, 2024
 *      Author: USER
 */

#include "keypad.h"

static uint8_t keypad[4][4] = {
  {1, 2, 3, 12}, // 12 represents A
  {4, 5, 6, 13}, // 13 represents B
  {7, 8, 9, 14}, // 14 represents C
  {10, 0, 11, 15} // 10 = *, 11 = #, 15 = D
};

void keypad_init(void) {

  RCC -> AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
  // Configure PC4-PC7 as outputs
  GPIOC->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
  GPIOC->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0);

  // Configure PC0-PC3 as inputs
  GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);

  // Configure PC0-PC3 as pull-down resistors
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD3_1);

  // Configure PC4-PC7 as push-pull outputs
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);

  //All of them are slow
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1 | GPIO_OSPEEDR_OSPEED2
		  | GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5
		  | GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);

  // Set PC4-PC7 high initially, all COLS high
  GPIOC->ODR &= (COL_PORT);
}

uint8_t getKeypadResult(uint8_t arr) {
    switch (arr) {
        case 0x01: return 0; // BIN 0001
        case 0x02: return 1; // BIN 0010
        case 0x04: return 2; // BIN 0100
        case 0x08: return 3; // BIN 1000
        default:
        	return -1; // Invalid or no key pressed
    }
}

int8_t getKeypadNumber(void){

	//Detect if a button is pressed
	if((GPIOC->IDR & ROW_PORT) != 0){
		for(uint32_t col = 0; col < NUM_COLS; col++){
			// Clear all columns then set them high one by one
			GPIOC->ODR &= ~(COL_PORT);
			GPIOC->ODR |= (1 << (col + 4)); // +4 because GPIOC PC4 - PC7 is used

			if((GPIOC->IDR & ROW_PORT) != 0){
				int colRes = getKeypadResult(1 << col);
				int rowRes = getKeypadResult(GPIOC->IDR & ROW_PORT);
				//If pressing to at once or let go in the middle
				if(colRes == -1 || rowRes == -1){
					return -1;
				}
				// reset high for next press
				GPIOC->ODR |= COL_PORT;
				return keypad[rowRes][colRes];
			}
		}
	}
	// reset high for next press
	GPIOC->ODR |= (COL_PORT);
	return -1;
}
