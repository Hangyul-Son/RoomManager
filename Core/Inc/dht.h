#ifndef __DHT_H
#define	__DHT_H

#include "stm32f1xx_hal.h"

uint16_t DHT_ReadSensor ( void );
uint16_t DHT_ProcessSensorData ( uint16_t dhtData );
uint16_t DHT_GetTemperature ( void );
uint16_t DHT_GetHumidity ( void ); 

#endif
