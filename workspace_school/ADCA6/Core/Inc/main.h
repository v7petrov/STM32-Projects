#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "USART.h"
#define sample_size 20	// sampling 20 times
#define MAX_VOLT 3300	// Max voltage in mV
#define MAX_DIGITAL 4095// Max voltage in 12 bit resolution
#define CHANNEL5 5		// Channel5 register setting
#define CONV_SLOPE 808	// These two values are calculated
#define CONV_OFFSET 699	// by using	linear approximation
#define INIT_DELAY 50000// 20 us delay for ADC init
#define LONG_DELAY 200000 // Long enough for us to see the USART terminal
#define OFF 0
#define STRING_length 6		// Volts w/ 3 decimals in includng null term + dec. point
#define THOUSANDS 1000		// Thousandths
#define HUNDREDS 100		// Hundreths
#define TENS 10				// Tenths

//#define digital_conv(digitval) ((digitval * (MAX_VOLT))/MAX_DIGITAL) (calibration eq)
#define digital_conv(digitval) (digitval * CONV_SLOPE - CONV_OFFSET)
#define modulo_ten(digit) (digit % 10)	// To get that digit of the number

void PrintMinMaxAvg(void);
void ADC1_2_IRQHandler(void);
char* DigitToStr(uint32_t digit);
void ADC_init(void);
void Error_Handler(void);
void ADC_delay(void);
void USART_delay(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
