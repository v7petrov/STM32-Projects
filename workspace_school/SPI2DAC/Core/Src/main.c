#include "main.h"

// Look-up table for keypad values
static uint8_t keypad[4][4] = {
  {1, 2, 3, 12}, // 12 represents A
  {4, 5, 6, 13}, // 13 represents B
  {7, 8, 9, 14}, // 14 represents C
  {10, 0, 11, 15} // 10 = *, 11 = #, 15 = D
};

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  //uses PA4 as NCS, PA5 as SCK, PA7 as PICO, and GPIOC (0 - 7) for keypad
  DAC_init();
  keypad_init();

  while (1)
  {
	  uint8_t presses = 0;
	  uint16_t millivolts = 0;

	  while(presses < 3){	// 3 numbers
		  int8_t number = getKeypadNumber();
		  if(number != -1){
			  //whenever keypad pressed, shift with new digit
			  millivolts = millivolts * 10 + number;

			  presses++;
			  while(getKeypadNumber() != -1);	// debounce keypad press
		  }
	  }

	  millivolts = volt_converter(millivolts);	// add FIFO mask and convert to 12 bit
	  DAC_send(millivolts);						// send to DAC
  }

}

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

void DAC_send(uint16_t num) {
    //send data
    SPI1->DR = num;
    // Wait until transmission is complete and SPI is not busy
    while (!(SPI1->SR & SPI_SR_TXE));  // Wait for TXE (Transmit buffer empty)
    while (SPI1->SR & SPI_SR_BSY);     // Wait until SPI is no longer busy
}

uint16_t volt_converter(uint16_t voltage) {	// max voltage is 330, max DAC value is 4095

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

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
