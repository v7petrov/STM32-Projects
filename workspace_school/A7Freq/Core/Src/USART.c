/*
 * USART.c
 *
 *  Created on: Nov 1, 2024
 *      Author: USER
 */

#include "USART.h"

void UART_init(void){

	// clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	//enable interrupts
	USART2->CR1 |= (USART_CR1_RXNEIE);
	//we sample over by 16
	USART2->CR1 &= ~USART_CR1_OVER8;
	//2 0s stops the transfer
	USART2->CR2 &= ~USART_CR2_STOP;
	//baud rate = SYSCLK/BR. over sample by 16 so this is it
	USART2->BRR = BR_DIV;
	// enable USART
	USART2->CR1 |= USART_CR1_UE;
	//enable transmit and receive
	USART2->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

	//enable gpio clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	//set GPIO PA2, 3 to AF (alternate function) for USART2,
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOA->MODER |= (AF << GPIO_MODER_MODE2_Pos | AF << GPIO_MODER_MODE3_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);

	//as described above but with max speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3);
	GPIOA->AFR[0] |= (USART_AF << GPIO_AFRL_AFSEL2_Pos | USART_AF << GPIO_AFRL_AFSEL3_Pos);
}

void USART_print(char* string){

	uint8_t i = 0;

	while(string[i] != 0){
		// Write to the USART_TDR if TXE is on
		while(!(USART2->ISR & USART_ISR_TXE));

		USART2->TDR = string[i];
		i++;
	}

}

void USART_ESC(char *string){
	uint8_t i = 0;

	// sends esc press
	while(!(USART2->ISR & USART_ISR_TXE));
	USART2->TDR = ESC_KEY;

	// sends rest of string
	while(string[i] != 0){
		// waits until transmit is finished
		while(!(USART2->ISR & USART_ISR_TXE));
		// loads the character into the transmit register
		USART2->TDR = string[i];
		i++;
	}
}

// Converts a number between 0 - 9999 to a string
char* DigitToStr(uint32_t digit) {
    static char result[STRING_length];

    // Convert each digit to a character and place in result string
    result[0] = (digit / THOUSANDS) + '0';          // Thousands
    result[1] = (modulo_ten(digit / HUNDREDS)) + '0';    // Hundreds place
    result[2] = (modulo_ten(digit / TENS)) + '0';     // Tens place
    result[3] = (modulo_ten(digit)) + '0';            // Units place
    result[4] = '\0';                          // Null terminator

    return result;
}
