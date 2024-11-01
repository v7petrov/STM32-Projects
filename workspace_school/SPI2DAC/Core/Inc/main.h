/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#define DAC_MAX 4095
#define VOLT_MAX 330
#define DAC_MASK 0x3000

#define WEIGHTS 0.993

#define NUM_ROWS 4
#define NUM_COLS 4
#define ROW_PORT (GPIO_IDR_ID0 | GPIO_IDR_ID1 | GPIO_IDR_ID2 | GPIO_IDR_ID3)
#define COL_PORT (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD7)

void SystemClock_Config(void);
void DAC_init(void);
void DAC_send(uint16_t);
void keypad_init(void);
uint8_t getKeypadResult(uint8_t);
int8_t getKeypadNumber(void);
uint16_t volt_converter (uint16_t);

void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
