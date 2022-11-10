#ifndef __DHT_H
#define	__DHT_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

void DHT_INIT(GPIO_TypeDef* DataPort, uint16_t DataPin);
void DHT_Start(void);
void DHT_ReadSensor (uint8_t* data);
bool DHT_ProcessSensorData (uint8_t* data, float *Temp, float *Humidity );
bool DHT_GetTemperatureAndHumidity (float *Temp, float *Humidity);

#endif
