#include "main.h"
#define NUM_ROWS 4
#define NUM_COLS 4
#define ROW_PORT (GPIO_IDR_ID0 | GPIO_IDR_ID1 | GPIO_IDR_ID2 | GPIO_IDR_ID3)
#define COL_PORT (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD7)
#define LED_PORT (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD7)
void SystemClock_Config(void);

//DEMO VIDEO: https://youtube.com/shorts/5qPlSSoLN4w?si=dc9cr0x66fUFotGn

// Look-up table for keypad values
static uint32_t keypad[4][4] = {
  {1, 2, 3, 12}, // 12 represents A
  {4, 5, 6, 13}, // 13 represents B
  {7, 8, 9, 14}, // 14 represents C
  {10, 0, 11, 15} // 10 = *, 11 = #, 15 = D
};


void keypad_init(void) {
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

uint32_t getKeypadResult(uint32_t arr) {
    switch (arr) {
        case 0x0001: return 0; // BIN 0001
        case 0x0002: return 1; // BIN 0010
        case 0x0004: return 2; // BIN 0100
        case 0x0008: return 3; // BIN 1000
        default:
        	return -1; // Invalid or no key pressed
    }
}

uint32_t getKeypadNumber(void){

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



int main(void)
{
  // System init
  SystemClock_Config();

  // Enable LED clock (GPIOA)
  RCC -> AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);

  // Enable KEYPAD clock (GPIOC)
  RCC -> AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);

  // LED outputs for GPIOA PA4 - PA7 (output, slow, no push/pull)
  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
  GPIOA->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0);
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5
		  | GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
  // Initialize keypad (uses GPIOC)
  keypad_init();

  while (1)
  {

    uint32_t number = getKeypadNumber();

    // 12 is A, 13 is B, 14 is C, 15 is D, 10 is *, 11 is #
    if(number != -1)  // If key pressed, show number on LEDs
    {
     	GPIOA->ODR &= ~LED_PORT;
    	GPIOA->ODR |= (number << 4);


    }
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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
	  break;
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
