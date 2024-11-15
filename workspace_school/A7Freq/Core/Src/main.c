#include "main.h"

void SystemClock_Config(void);

uint8_t flag = 0;				// Global flags
uint16_t ADC_val = 0;			// Global ADC_val
uint16_t samples[sample_size];	// Global sample array
uint16_t sample_index = 0;		// Index for the array

uint16_t DC_offset;

int main(void)
{
    // Initialize the hardware abstraction layer
    HAL_Init();

    // Configure system clock
    SystemClock_Config();

    // Initialize UART and ADC peripherals
    UART_init();
    ADC_init();
    TIM2_init();

    // Main loop
    while (1)
    {
    	// Check if the flag is set indicating a set of ADC conversions are complete
		if (!flag) {
			// CALCULATE AND PRINT THE FREQUENCY
			USART_ESC(CLR_KEY);
			USART_ESC(RST_KEY);
			USART_print(DigitToStr(calculateFreq()));
			USART_delay();	// make frequency visible
			flag = 1;
		}
    }
}

// function that uses a lot of math from the sample waveform to calculate the frequency
uint16_t calculateFreq(void){

	// First we calculate the DC offset in order to bias it out. Sine wave so it's usually the average
	uint32_t sum = 0;
	uint32_t DC_offset;

	for (int i = 0; i < sample_size; i++) {
		sum += samples[i];
	}
	DC_offset = sum / sample_size;

	// We count how many times the samples crosses the DC_offset in the second (2k samples @ 2kHz)
	uint16_t crossings = 0;
	 for (int i = 1; i < sample_size; i++) {
		if ((samples[i - 1] > DC_offset && samples[i] <= DC_offset) ||
			(samples[i - 1] < DC_offset && samples[i] >= DC_offset)) {
			crossings++;
		}
	 }
	 	// 2 crossings happen per period. This gives us the frequency of the waveform
	    uint16_t frequency = crossings / 2;

	    return frequency;

}

void TIM2_IRQHandler(void) {	// Every 500us, sample waveform (cuz we're awesome)

	if(TIM2->SR & TIM_SR_UIF){
		if(flag){
			// Start an ADC conversion
			ADC1->CR |= (ADC_CR_ADSTART);
		}

		// Clear flag
		TIM2->SR &= ~(TIM_SR_UIF);
	}

}

void ADC1_2_IRQHandler(void) {
    // Check if ADC end-of-conversion flag is set
    if (ADC1->ISR & ADC_ISR_EOC) {
        samples[sample_index] = ADC1->DR; // Read conversion result from ADC data register
        sample_index++;
        if(sample_index == sample_size){
        	flag = OFF;
        	sample_index = 0;
        }
    }

    // Clear the end-of-conversion flag
    ADC1->ISR &= ~ADC_ISR_EOC;
}

void TIM2_init(void) {
	// TIM2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Clock frequency is 24MHz / PSC to equal 10kHz counter freq(PSC = 2400)
	TIM2->PSC = PRSCL - 1;

	// Set ARR to 5 for 2kHz frequency (10kHz/2kHz)
	TIM2->ARR = ARR_VAL - 1;

	// Enable update interrupt and disable CCR interrupt
	TIM2->DIER |= (TIM_DIER_UIE);
	TIM2->DIER &= ~TIM_DIER_CC1IE;

	// Clear flag stuff
	TIM2->SR &= ~(TIM_SR_UIF);

	// Count up mode
	TIM2->CR1 &= ~(TIM_CR1_DIR);

	// Enable TIM2 interrupt in NVIC
	NVIC->ISER[0] = (1 << TIM2_IRQn);

	// Enable global interrupts
	__enable_irq();

	// Enable timer
	TIM2->CR1 |= TIM_CR1_CEN;

}

void ADC_init(void) {
    // Enable ADC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    // Set ADC clock to HCLK/1 synchronous mode
    ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos);

    // Power up ADC voltage regulator
    ADC1->CR &= ~(ADC_CR_DEEPPWD);
    ADC1->CR |= (ADC_CR_ADVREGEN);

    // Wait for ADC voltage regulator to stabilize
    ADC_delay();

    // Configure ADC for single-ended mode on channel 5 (PA0)
    ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

    // Calibrate the ADC
    ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
    ADC1->CR |= ADC_CR_ADCAL; // Start calibration
    while (ADC1->CR & ADC_CR_ADCAL); // Wait for calibration to finish

    // Enable ADC and wait for it to be ready
    ADC1->ISR |= (ADC_ISR_ADRDY);
    ADC1->CR |= (ADC_CR_ADEN);
    while (!(ADC1->ISR & ADC_ISR_ADRDY));

    // Set up ADC to sample channel 5 once in a sequence
    ADC1->SQR1 = (CHANNEL5 << ADC_SQR1_SQ1_Pos);
    ADC1->CFGR = 0;

    // Configure sample time
    ADC1->SMPR1 = ~(ADC_SMPR1_SMP5);
    ADC1->SMPR1 = (7 << ADC_SMPR1_SMP5_Pos);

    // Enable interrupts on end of conversion
    ADC1->IER |= (ADC_IER_EOC);
    ADC1->ISR |= (ADC_ISR_EOC);

    // Configure GPIO for channel 5		PA0
    RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->ASCR    |= (GPIO_ASCR_ASC0); // Connect analog switch to ADC input
    GPIOA->MODER   &= ~(GPIO_MODER_MODE0);
    GPIOA->MODER   |= (GPIO_MODER_MODE0); 	// Analog mode
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0);	// Max speed

    // Enable interrupt in NVIC
    NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));
}

void ADC_delay(void) {   // Delay for ADC setup
    for (uint32_t i = 0; i < INIT_DELAY; i++);
}

void USART_delay(void) { // Delay to allow USART output to be visible
    for (uint32_t i = 0; i < LONG_DELAY; i++);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    // Initializes the RCC Oscillators
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Initialize CPU, AHB and APB clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    // User can add custom error handling here
}
#endif /* USE_FULL_ASSERT */
