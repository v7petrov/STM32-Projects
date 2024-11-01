/*
 * DAC.c
 *
 *  Created on: Oct 26, 2024
 *      Author: USER
 */
#include "DAC.h"

void DAC_init(void){
    // Enable GPIOA and SPI1 clock
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
    RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);

    // Configure PA4 (NCS), PA5 (SCK) and PA7 (MOSI) as AF
    GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE7);
    GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1);

    // Pins are push/pull, fast speed, and have no pull up/pull down
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT7);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED7);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD7);

    // Set PA4, PA5 and PA7 to Alternate Function 5 for SPI
	GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
	GPIOA -> AFR[0] |= (GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_2);
	GPIOA -> AFR[0] |= (GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2);
	GPIOA -> AFR[0] |= (GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_2);

    //SPI1 CR1 register as per chip instructions
    SPI1->CR1 &= ~SPI_CR1_BR;		// Configure Baud rate f/2
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);  	// Clock: 0 polarity 0 phase

    SPI1->CR1 &= ~SPI_CR1_RXONLY; // read only, simplex, etc
    SPI1->CR1 |= SPI_CR1_BIDIMODE;
    SPI1->CR1 |= SPI_CR1_BIDIOE;

    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;           // MSB first

    SPI1->CR1 &= ~(SPI_CR1_SSM);     // Hardware NCS

    SPI1->CR1 |= SPI_CR1_MSTR;                  // Controller is controller


    SPI1->CR2 |= (SPI_CR2_DS);       			    // 16 bit

    SPI1->CR2 |= SPI_CR2_SSOE;					// single controller

    SPI1->CR2 |= SPI_CR2_NSSP;					//generate NSS pulse

    SPI1->CR2 &= ~SPI_CR2_FRXTH;					//FRXTH off bc 16 bit

	//enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;

}

void DAC_write(uint16_t num) {
    //send data
    SPI1->DR = num;
    // Wait until transmission is complete and SPI is not busy
    while (!(SPI1->SR & SPI_SR_TXE));  // Wait for TXE (Transmit buffer empty)
    while (SPI1->SR & SPI_SR_BSY);     // Wait until SPI is no longer busy
}

uint16_t DAC_volt_conv(uint16_t voltage) {	// max voltage is 3300, max DAC value is 4095

    if (voltage > VOLT_MAX) {
        voltage = VOLT_MAX;  // Max voltage limit
    }

    // Convert voltage to a 12-bit DAC value
    uint16_t dacValue = (voltage * DAC_MAX) / VOLT_MAX;
    dacValue = dacValue * WEIGHTS;	// value based on DMM/outside factors

    // Apply DAC command bits (FIFO mask) 0011 XXXX XXXX XXXX
    dacValue |= DAC_MASK;

    return dacValue;
}
