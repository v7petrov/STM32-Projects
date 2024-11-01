

#include "main.h"


void SystemClock_Config(void);


int main(void)
{


 HAL_Init();


 SystemClock_Config();



//enable the clock for GPIOC
 RCC -> AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);


 //configure P C0, C1, C2, C3

 GPIOC -> MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3); //clear mode register pins 0 - 3
 GPIOC -> MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0); //set mode to 01 on pins 0 - 3, output
 GPIOC -> OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3); //set type to 0 on pins 0 -3
 GPIOC -> OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1 | GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3); //speed is 0 on all 3 so hopefully blinking shows up
 GPIOC -> PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3); //no pull up/down on outputs




 while (1)
 {
   /* USER CODE END WHILE */
	for(int i = 0; i < 16; i++)
	{
		for(int j = 50000; j>0; j--);	//delay
		// REGISTER OUTPUT FOR GPIOC     CLEAR THE 1ST 4 PINS                                   0R IT WITH i, which is the counting variable
		GPIOC->ODR = (GPIOC->ODR & ~(GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3)) | (i & 0xF); //we only want to change the last 4 as a counter

		// https://youtu.be/kCgib57cIhY?si=vSzidgFX_muMYSbB DEMO OF BLINKER

	}
   /* USER CODE BEGIN 3 */
 }
 /* USER CODE END 3 */
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
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
