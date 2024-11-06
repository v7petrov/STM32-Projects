/*
 * UART.h
 *
 *  Created on: Nov 1, 2024
 *      Author: USER
 */

#ifndef INC_USART_H_
#define INC_USART_H_
#include "stm32l4xx_hal.h"

#define USART_INTERRUPT_EN 0x1F
#define AF 0x2
#define USART_AF 0x7
#define BR_DIV 208		// 24MHz/115.2kbps
#define ESC_KEY 0x1b

#define B_KEY "[34m"
#define G_KEY "[32m"
#define R_KEY "[31m"
#define W_KEY "[37m"

#define CLR_KEY "[2J"
#define RST_KEY "[H"

void UART_init(void);
void USART_print(char* string);
void USART_ESC(char *string);

#endif /* INC_USART_H_ */
