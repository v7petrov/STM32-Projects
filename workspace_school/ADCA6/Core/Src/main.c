#include "main.h"

void SystemClock_Config(void);

uint8_t flag = 0;				// Global flags
uint16_t ADC_val = 0;			// Global ADC_val
uint16_t samples[sample_size];	// Global sample array
uint8_t sample_index = 0;		// Index for the array

int main(void)
{
    // Initialize the hardware abstraction layer
    HAL_Init();

    // Configure system clock
    SystemClock_Config();

    // Initialize UART and ADC peripherals
    UART_init();
    ADC_init();

    // Main loop
    while (1)
    {
        // Check if the flag is set indicating an ADC conversion is complete
        if (flag) {
            // Store the ADC value in the samples array
            samples[sample_index] = ADC_val;
            sample_index++;

            // Check if sample array is filled
            if (sample_index == sample_size) {
                // Process and print the minimum, maximum, and average values
                PrintMinMaxAvg();
                sample_index = 0; // Reset index for the next set of samples
            }

            // Clear the flag and start a new ADC conversion
            flag = OFF;
            ADC1->CR |= (ADC_CR_ADSTART); // Start a conversion
        }
    }
}

void PrintMinMaxAvg(void) {
    uint16_t min = MAX_DIGITAL;
    uint16_t max = OFF;
    uint32_t sum = 0;
    uint16_t avg;

    // Calculate min, max, and sum of all sampled values
    for (uint8_t i = 0; i < sample_size; i++) {
        if (samples[i] < min) {
            min = samples[i];	// New minimum
        }

        if (samples[i] > max) {	// New maximum
            max = samples[i];
        }

        sum += samples[i];
    }

    // Calculate average of sampled values
    avg = sum / sample_size;

    // Convert digital values to voltages in mV
    min = digital_conv(min);
    max = digital_conv(max);
    avg = digital_conv(avg);

    // Clear the screen and reset cursor
    USART_ESC(CLR_KEY);
    USART_ESC(RST_KEY);

    // Print minimum, maximum, and average values to the terminal
    USART_print("Minimum: ");
    USART_print(DigitToStr(min));
    USART_print("     ");
    USART_print("Maximum: ");
    USART_print(DigitToStr(max));
    USART_print("     ");
    USART_print("Average: ");
    USART_print(DigitToStr(avg));
    USART_print("     ");

    // Add delay to allow values to be read on the terminal
    USART_delay();
}

char* DigitToStr(uint32_t digit) {
    static char result[STRING_length];

    // Scale down to get a 3.3V equivalent
    digit = digit / THOUSANDS;

    // Convert each digit to a character and place in result string
    result[0] = (digit / THOUSANDS) + '0';          // Thousands
    result[1] = '.';									// Decimal point
    result[2] = (modulo_ten(digit / HUNDREDS)) + '0';    // Hundreds place
    result[3] = (modulo_ten(digit / TENS)) + '0';     // Tens place
    result[4] = (modulo_ten(digit)) + '0';            // Units place
    result[5] = '\0';                          // Null terminator

    return result;
}

void ADC1_2_IRQHandler(void) {
    // Check if ADC end-of-conversion flag is set
    if (ADC1->ISR & ADC_ISR_EOC) {
        ADC_val = ADC1->DR; // Read conversion result from ADC data register
        flag = 1;           // Set global flag to indicate a new value is ready
    }

    // Clear the end-of-conversion flag
    ADC1->ISR &= ~ADC_ISR_EOC;
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

    // Enable interrupts on end of conversion
    ADC1->IER |= (ADC_IER_EOC);
    ADC1->ISR |= (ADC_ISR_EOC);

    // Configure GPIO for channel 5
    RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->ASCR    |= (GPIO_ASCR_ASC0); // Connect analog switch to ADC input
    GPIOA->MODER   &= ~(GPIO_MODER_MODE0);
    GPIOA->MODER   |= (GPIO_MODER_MODE0); 	// Analog mode
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0);	// Max speed

    // Enable interrupt in NVIC
    NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));
    __enable_irq();

    // Start an ADC conversion
    ADC1->CR |= (ADC_CR_ADSTART);
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
