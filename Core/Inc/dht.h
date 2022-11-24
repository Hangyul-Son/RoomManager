#ifndef __DHT_H
#define	__DHT_H

#include "stm32f1xx_hal.h"
#include "utility.h"

void DHT_Init(GPIO_TypeDef* DataPort, uint16_t DataPin);
void DHT_Start(void);
void DHT_ReadSensor (uint8_t* data);
bool DHT_ProcessSensorData (uint8_t* data, float *Temp, float *Humidity );
bool DHT_GetTemperatureAndHumidity (float *Temp, float *Humidity);
float DHT_Get_DI();
void DHT_Display();
bool DHT_Check();


#endif
