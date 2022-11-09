#ifndef __FAN_H
#define	__FAN_H

#include "stm32f1xx_hal.h"
#include "dht.h"

void FAN_Rotate ( uint16_t speed );
uint16_t FAN_GetSpeed ( uint16_t dhtData)

#endif