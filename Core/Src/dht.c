

#include "dht.h"
#include "main.h"
#include "lcd.h"


float Temperature, Humidity;

GPIO_TypeDef* DHT_PORT;
uint16_t DHT_PIN;

void DHT_Init(GPIO_TypeDef* DataPort, uint16_t DataPin)
{
	DHT_PORT = DataPort;
	DHT_PIN = DataPin;
}

void DHT_Test(void){
	HAL_GPIO_WritePin(DHT_PORT,DHT_PIN, GPIO_PIN_SET);
}
void DHT_Start (void)
{
	//Change data pin mode to OUTPUT
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
	//Put pin LOW

	HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
	//500uSec delay
	DelayMicroSeconds(1200);
	//Bring pin HIGH
	HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
	//30 uSec delay
	DelayMicroSeconds(30);
	//Set pin as input
	GPIO_InitStruct.Pin = DHT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
//	LCD_DrawChar(0,0,'A');
}


void DHT_ReadSensor (uint8_t* data)
{

    uint32_t rawBits = 0UL;
    uint8_t checksumBits = 0UL;

    DelayMicroSeconds(40);

    while(!(1&HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)));
    while((1&HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)));
    for(int8_t i=31; i>=0; i--)
    {
        while(!(1&HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)));
        DelayMicroSeconds(40);
        if((1&HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)))
        {
            rawBits |= (1UL << i);
        }
        while((1&HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)));
    }

    for(int8_t i=7; i>=0; i--)
    {
        while(!(1&HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)));
        DelayMicroSeconds(40);
        if((1&HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)))
        {
            checksumBits |= (1UL << i);
        }
        while((1&HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)));
    }


    //Copy raw data to array of bytes
    data[0] = (rawBits>>24)&0xFF;
    data[1] = (rawBits>>16)&0xFF;
    data[2] = (rawBits>>8)&0xFF;
    data[3] = (rawBits>>0)&0xFF;
    data[4] = (checksumBits)&0xFF;
}

bool DHT_ProcessSensorData (uint8_t* data, float *Temp, float *Humidity)
{
	uint16_t Temp16, Humid16;
	uint8_t myChecksum = 0;
	for(uint8_t k=0; k<4; k++)
	{
		myChecksum += data[k];
	}
	if(myChecksum == data[4])
	{
		Temp16 = (data[2] <<8) | data[3];
		Humid16 = (data[0] <<8) | data[1];

		*Temp = Temp16/10.0f;
		*Humidity = Humid16/10.0f;
		return 1;
	}
	else
	{
		return 0;
	}
}



bool DHT_GetTemperatureAndHumidity (float *Temp, float *Humidity)
{

    uint8_t rawData[6];

    DHT_Start();
    DHT_ReadSensor(rawData);

    return DHT_ProcessSensorData(rawData, Temp, Humidity);

}

float DHT_Get_DI()
{
	//Calculate Discomfort Index
	return GetDiscomfortIndex(&Temperature, &Humidity);
}

void DHT_Display()
{
	LCD_Clear(0,0,240,320, WHITE);
	char temperature[5], humidity[5];
	sprintf(temperature, "%.2f", Temperature);
	sprintf(humidity, "%.2f", Humidity);
	LCD_DrawString(60,100, temperature);
	LCD_DrawString(120,100, humidity);
}


bool DHT_Check()
{
	return DHT_GetTemperatureAndHumidity(&Temperature, &Humidity);


}
