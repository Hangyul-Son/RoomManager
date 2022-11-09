#include "dht.h"


// Left: 0, Right: 1
void FAN_Rotate ( uint16_t direction ) {
	if (direction == 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	}
	else if(direction == 1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
}

uint16_t FAN_GetSpeed ( uint16_t dhtData) {
	return 0;
}
