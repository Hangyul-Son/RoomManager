#ifndef __FAN_H
#define	__FAN_H

#include "stm32f1xx_hal.h"

void FAN_Init(GPIO_TypeDef* DataPortA, uint16_t DataPinA, GPIO_TypeDef* DataPortB, uint16_t DataPinB);
void FAN_Rotate (char dir, int speed);
void FAN_Off ();


#endif
