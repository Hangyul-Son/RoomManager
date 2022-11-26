#include "fan.h"

TIM_HandleTypeDef *htim;
uint32_t Channel;

void FAN_Init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	htim = htim;
	Channel = Channel;
}

void FAN_Rotate (int speed)
{
	user_pwm_setvalue(htim, Channel, speed);
}

void FAN_Off ()
{
	user_pwm_setvalue(htim, Channel, 0);
}
