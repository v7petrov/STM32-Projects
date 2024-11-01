#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <math.h>

#define LUT_SIZE 6000
#define DC_OFFSET 1500
#define Vp 1500
#define Vpp 3000
#define PI 3.14
#define PWM 9
#define ON 1
#define OFF 0
#define NO_PRESS -1

#define ARR_VAL 136

void LUT_init(void);
void TIM2_init(void);
void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
