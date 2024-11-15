#include "main.h"
#include "DAC.h"	// PA4 - NCS, PA5 - SCK, PA7 - PICO
#include "keypad.h"	// KEYPAD PC0 - 7

typedef enum{SQR, SIN, TRI, SAW} WAVE;
static WAVE curr_state = SQR;			// start waveform at square
static uint8_t curr_freq = 1; 			// start frequency at 100Hz, var * 100 = Hz
static uint16_t curr_index = 0;			// global index, index through the LUT at this point
static uint8_t curr_PWM = 4;			// start at 50% duty cycle, where var + 1 * 10  is duty%

void SystemClock_Config(void);

// LUT for waves as an array, index through this
static uint16_t SIN_WAVE[LUT_SIZE];
static uint16_t TRI_WAVE[LUT_SIZE];
static uint16_t SAW_WAVE[LUT_SIZE];
static uint8_t SQR_WAVE[PWM - 1][LUT_SIZE];


int main(void)
{

  HAL_Init();
  keypad_init();
  DAC_init();
  LUT_init();
  TIM2_init();	// Timer init is last because everything has to be ready before interrupts are enabled in here.

  SystemClock_Config();

  while (1)
  {
	  while(getKeypadNumber() == NO_PRESS);	// 2 STAGE DEBOUNCE: Stop while there is no press
	  int8_t number = getKeypadNumber();
	  while(getKeypadNumber() != NO_PRESS);	// Don't exit until the press is released

	  // set a different state depending on keypad press
	  switch (number){
		  case (0):
		  	  // 50% duty cycle if a square wave
		  	  if(curr_state == SQR){
		  		  curr_PWM = 4;
		  	  }
		  	  break;

		  case (1):
		  	  // 100 Hz frequency
		  	  curr_freq = 1;
		  	  break;
		  case (2):
		  	  // 200 Hz frequency
			  curr_freq = 2;
			  break;
		  case (3):
		  	  // 300 Hz frequency
			  curr_freq = 3;
			  break;
		  case (4):
		  	  // 400 Hz frequency
			  curr_freq = 4;
			  break;
		  case (5):
		  	  // 500 Hz frequency
			  curr_freq = 5;
			  break;
		  case (6):
		  	  // Sine wave
		  	  curr_state = SIN;
		  	  break;
		  case (7):
		  	  // Triangle wave
			  curr_state = TRI;
			  break;
		  case (8):
		  	  // Sawtooth wave
			  curr_state = SAW;
			  break;
		  case (9):
		  	  // Square wave
			  curr_state = SQR;
			  break;
		  case (10):
		  	  // -10% duty cycle
			  if(curr_state == SQR){
				  if(curr_PWM != 0) {
					 curr_PWM -= 1;
				  }

			  }
		  	  break;
		  case (11):
		  	  // +10% duty cycle
			  if(curr_state == SQR){
				  if(curr_PWM != PWM - 1) {
					  curr_PWM += 1;
				  }

			  }
		  	  break;
		  default:
			  // don't do anything
	  }

  }

}

void TIM2_IRQHandler(void) {
  	//IRQ handler will decide how and what to write to DAC
  	if(TIM2 -> SR & TIM_SR_UIF){ //happens when ARR hits the count
  		if(curr_index > LUT_SIZE){
  			curr_index = 0;
  		}
  		switch(curr_state) {
  			case SQR:
  				// the LUT we use is decided both on PWM and the index.
  				DAC_write(DAC_volt_conv((uint16_t)(SQR_WAVE[curr_PWM][curr_index]) * Vpp)); // the table either has a 1 or 0 to write to the DAC, so we convert it
  				break;
  			case SIN:
  				DAC_write(SIN_WAVE[curr_index]); // If we're on a SINE WAVE, we write from SINE LUT
  				break;
  			case TRI:
  				DAC_write(TRI_WAVE[curr_index]); // If we're on a TRIANGLE WAVE, we write from TRI LUT
  				break;
  			case SAW:
  				DAC_write(SAW_WAVE[curr_index]); // If we're on a SAWTOOTH WAVE, we write from SAW LUT
  				break;
  			default:
  				break;
  		}
  		curr_index += curr_freq;	// Index based on the frequency. This makes us go through the LUT faster and creates
  		TIM2 -> SR &= ~TIM_SR_UIF; //clear interrupt flag
  	}
}

void LUT_init(void)
{
	uint16_t half = LUT_SIZE / 2;
	for(uint16_t i = 0; i < LUT_SIZE; i++){
		// Generate LUT for different waveforms
		SIN_WAVE[i] = DAC_volt_conv((uint16_t)(Vp * sin(2*PI*i/LUT_SIZE) + DC_OFFSET));	// Equation to make equally spaced points for a period on a sinewave
		SAW_WAVE[i] = DAC_volt_conv((uint16_t)(Vpp*i / LUT_SIZE));						// Equation to make equally spaced points for a saw wave
		if(i < half){
			TRI_WAVE[i] = DAC_volt_conv((uint16_t)(Vpp*i / half));						// First half of a triangle wave goes from 0 to 3V
		}
		else{
			TRI_WAVE[i] = DAC_volt_conv((uint16_t)(Vpp - (Vpp*(i-half) / half)));		// second half of a triangle wave goes from 3V to 0
		}
		for(uint16_t j = 1; j <= PWM; j++) {	// Generate a table of 1s and 0s. The point at which 1s become 0s change based on the duty %
			if (i <= (j * LUT_SIZE)/10) {
			SQR_WAVE[j - 1][i] = ON;

			}
			else {
				SQR_WAVE[j - 1][i] = OFF;
			}
		}


	}
}

void TIM2_init(void) { // TIM2 and interrupt setup
	// TIM2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Turn off pre-scaler
	TIM2->PSC = 0;

	// Set ARR to calculated value
	TIM2->ARR = ARR_VAL - 1;

	// Set CCR1 to 400 - 1 for 5kHz 50% duty
//	TIM2->CCR1 = CCR1_VAL - 1;

	// enable overflow interrupt
	TIM2->DIER |= (TIM_DIER_UIE);

	// Disable CCR interrupt
	TIM2->DIER &= ~(TIM_DIER_CC1IE);

	// Clear flag stuff
	TIM2->SR &= ~TIM_SR_CC1IF;
	TIM2 -> SR &= ~TIM_SR_UIF;

	// Count up mode
	TIM2->CR1 &= ~(TIM_CR1_DIR);

	// Enable TIM2 interrupt in NVIC
	NVIC->ISER[0] = (1 << TIM2_IRQn);

	// Enable global interrupts
	__enable_irq();

	// Enable timer
	TIM2->CR1 |= TIM_CR1_CEN;
}

void SystemClock_Config(void)	// 80 MHZ SYSTEM CLOCK!!!!!!!!!
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
