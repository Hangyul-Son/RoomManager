#ifndef __UTILITY_H
#define	__UTILITY_H

#include "stm32f1xx_hal.h"


uint32_t DWT_Delay_Init(void);
void DelayMicroSeconds(volatile uint32_t au32_microseconds);

#endif
