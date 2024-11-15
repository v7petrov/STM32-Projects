#include "main.h"
#define USART_INTERRUPT_EN 0x1F
#define AF 0x2
#define USART_AF 0x7
#define BR_DIV 35		// 4MHz/115.2kbps
#define ESC_KEY 0x1b

#define B_KEY "[34m"
#define G_KEY "[32m"
#define R_KEY "[31m"
#define W_KEY "[37m"

#define CLR_KEY "[2J"

void SystemClock_Config(void);

void UART_init(void);
void USART2_IRQHandler(void);
void USART_print(char* string);
void USART_ESC(char *in_string);

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  UART_init();

  //wait for it to all work properly
  while(!(USART2->ISR & USART_ISR_TXE));

  USART_ESC(CLR_KEY);	// clear screen and reset cursor
  USART_ESC("[3B");		// move cursor 3 lines down
  USART_ESC("[5C");		// move cursor 5 spaces right
  USART_print("All good students read the");
  USART_ESC("[1B");		// move cursor 1 line down
  USART_ESC("[21D");	// move cursor 21 spaces left
  USART_print("Reference Manual");
  USART_ESC("[H");			// move cursor to top left
  USART_ESC("[0m");			// remove character attributes
  USART_print("Input: ");

  // enable interrupts in NVIC
  NVIC -> ISER[1] = (1 << (USART2_IRQn & USART_INTERRUPT_EN));

  while (1)
  {

  }

}

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

void USART2_IRQHandler(void){
	//
	if (USART2->ISR & USART_ISR_RXNE){
		switch(USART2->RDR){
			case 'B':
				USART_ESC(B_KEY);
				break;
			case 'G':
				USART_ESC(G_KEY);
				break;
			case 'R':
				USART_ESC(R_KEY);
				break;
			case 'W':
				USART_ESC(W_KEY);
				break;
			default:
				break;

	}
		// wait for USART transmit to be ready
		while(!(USART2->ISR & USART_ISR_TXE));
		USART2->TDR = USART2->RDR;
		// clear flag
		USART2->ISR &= ~(USART_ISR_RXNE);
	}
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
