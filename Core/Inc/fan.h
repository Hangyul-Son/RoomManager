#ifndef __FAN_H
#define	__FAN_H

#include "stm32f1xx_hal.h"
#include "main.h"
void FAN_Init(TIM_HandleTypeDef *htim, uint32_t Channel);
void FAN_Rotate (int speed);
void FAN_Off ();


#endif
