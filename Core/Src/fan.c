#include "fan.h"

GPIO_TypeDef* FAN_PORT_A;
uint16_t FAN_PIN_A;
GPIO_TypeDef* FAN_PORT_B;
uint16_t FAN_PIN_B;

void FAN_Init(GPIO_TypeDef* DataPortA, uint16_t DataPinA, GPIO_TypeDef* DataPortB, uint16_t DataPinB)
{
	FAN_PORT_A = DataPortA;
	FAN_PIN_A = DataPinA;
	FAN_PORT_B = DataPortB;
	FAN_PIN_B = DataPinB;
}

// Left: 0, Right: 1
void FAN_Rotate (char dir, int speed )
{
	if (dir == 'r'){
		HAL_GPIO_WritePin(FAN_PORT_A, FAN_PIN_A, GPIO_PIN_SET);
		HAL_GPIO_WritePin(FAN_PORT_B, FAN_PIN_B, GPIO_PIN_RESET);
	}
	else if(dir == 'l'){
		HAL_GPIO_WritePin(FAN_PORT_A, FAN_PIN_A, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FAN_PORT_B, FAN_PIN_B, GPIO_PIN_SET);
	}
}

void FAN_Off ()
{
	HAL_GPIO_WritePin(FAN_PORT_A, FAN_PIN_A, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FAN_PORT_B, FAN_PIN_B, GPIO_PIN_RESET);
}
