#ifndef __UTILITY_H
#define	__UTILITY_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define size_str 7

uint32_t DWT_Delay_Init(void);
void DelayMicroSeconds(volatile uint32_t au32_microseconds);
float GetDiscomfortIndex(float *Temperature, float* Humidity);


#endif
