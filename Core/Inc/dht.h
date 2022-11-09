#ifndef __DHT_H
#define	__DHT_H

#include "stm32f1xx_hal.h"

void DHT_Start(void);
void DHT_ReadSensor (uint8_t* data);
bool DHT_ProcessSensorData (uint8_t* data, float *Temp, float *Humidity );
bool DHT_GetTemperatureAndHumidity (float *Temp, float *Humidity);

#endif
