#include "main.h"
#include "DAC.h"
#include <stdint.h>
#define ARR_VAL 0 // Run TIM2 continuously (will go to 0xffffffff)
#define CCR1_VAL 64 // 50% duty cycle 5kHz (interrupts every 1/10kHz)

void SystemClock_Config(void);
void GPIO_Config(void);
void TIM_Config(void);


int main(void) {
    // Initialize HAL and system clock
    HAL_Init();
    SystemClock_Config();

    // GPIO and TIM configs
    GPIO_Config();
    TIM_Config();
    DAC_init();

    // Infinite loop, ISR will handle timing
    while (1) {
    }
}

void TIM2_IRQHandler(void) {
//	 Start ISR timing signal
	GPIOC->ODR |= (GPIO_ODR_OD1);

	// Conditional check here unnecessary for our use
// case but increases portability, at cost of speed
	if (TIM2->SR & TIM_SR_CC1IF) {
		// Toggle signal
//		GPIOC->ODR ^= (GPIO_ODR_OD0);

		// Increment CCR1
//		TIM2->CCR1 += CCR1_VAL;

		DAC_write(0);

		// Clear flag
		TIM2->SR &= ~(TIM_SR_CC1IF);
	}
//	 Stop ISR timing signal
	GPIOC->ODR &= ~(GPIO_ODR_OD1);
}

void GPIO_Config(void) { // GPIOC setup
	// GPIOC Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC0 as output
	GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1); //Clear PC0 modeR
	GPIOC->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0); //PC0 as output

	// OTYPER: Output Push-Pull (0)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);

	// OSPEEDR: Very High Speed (11)
	GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);

	// PUPDR: Neither (00)
	GPIOC->PUPDR &=  ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);

	// Default output pin high
	GPIOC->ODR |= (GPIO_ODR_OD0);

	// Default ISR timing pin low
	GPIOC->ODR &= ~(GPIO_ODR_OD1);

	// Given code to output MCO to PA8
	// Enable MCO, select MSI (4 MHz source)
	RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));

	// Configure MCO output on PA8
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER   &= ~(GPIO_MODER_MODE8);		// alternate function
	GPIOA->MODER   |=  (2 << GPIO_MODER_MODE8_Pos);
	GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT8);		// Push-pull output
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD8);		// no resistor
	GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8);		// high speed
	GPIOA->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);		// select MCO function
}

void TIM_Config(void) { // TIM2 and interrupt setup
	// TIM2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Turn off pre-scaler
	TIM2->PSC = 0;

	// Set ARR to 0xffffffff (run to max)
	TIM2->ARR = ARR_VAL - 1;

	// Set CCR1 to 400 - 1 for 5kHz 50% duty
	TIM2->CCR1 = CCR1_VAL - 1;

	// Disable overflow interrupt
	TIM2->DIER &= ~(TIM_DIER_UIE);

	// Enable CCR interrupt
	TIM2->DIER |= (TIM_DIER_CC1IE);

	// Clear flag stuff
	TIM2->SR &= ~(TIM_SR_CC1IF);

	// Count up mode
	TIM2->CR1 &= ~(TIM_CR1_DIR);

	// Enable TIM2 interrupt in NVIC
	NVIC->ISER[0] = (1 << TIM2_IRQn);

	// Enable global interrupts
	__enable_irq();

	// Enable timer
	TIM2->CR1 |= TIM_CR1_CEN;
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
