
void DelayMicroSeconds(uint32_t uSec)
{
    uint32_t uSecVar = uSec;
    uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
    while(uSecVar--);
}

void DHT_Start (void)
{
	//Change data pin mode to OUTPUT
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//Put pin LOW
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	//500uSec delay
	DelayMicroSeconds(500);
	//Bring pin HIGH
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	//30 uSec delay
	DelayMicroSeconds(30);
	//Set pin as input
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void DHT_ReadSensor (uint8_t* data)
{

    uint32_t rawBits = 0UL;
    uint8_t checksumBits=0;

    DelayMicroSeconds(40);

    while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));

    for(int8_t i=31; i>=0; i--)
    {
        while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
        DelayMicroSeconds(40);
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
        {
            rawBits |= (1UL << i);
        }
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
    }

    for(int8_t i=7; i>=0; i--)
    {
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
        DelayMicroSeconds(40);
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
        {
            checksumBits |= (1UL << i);
        }
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
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
	return 0
}



bool DHT_GetTemperatureAndHumidity (float *Temp, float *Humidity)
{

    uint8_t rawData[6];

    DHT_Start();

    DHT_ReadSensor(rawData);

    return DHT_ProcessSensorData(rawData, Temp, Humidity)

}
