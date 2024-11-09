#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "USART.h"

#define COMP_INTERRUPT 0x1F
#define ARR_VAL 5
#define CLKFREQ 24000000
#define sample_size 2000
#define OFF 0
#define PRSCL 2400
#define CHANNEL5 5
#define INIT_DELAY 50000// 20 us delay for ADC init
#define LONG_DELAY 200000 // Long enough for us to see the USART terminal

void ADC1_2_IRQHandler(void);
void ADC_init(void);
void Error_Handler(void);
void ADC_delay(void);
void USART_delay(void);
uint16_t calculateFreq(void);
void TIM2_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
