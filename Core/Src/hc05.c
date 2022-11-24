
#include "hc05.h"

bool fanSwitch;
bool screenSwitch;
bool alarmSwitch;

/*
 * Alarm
 * Alarm on and off.
 * Alarm time settting. Start time / End time.
 * Alarm sound(type of music) setting?
 * Alarm sound
 *
 * Fan
 * Fan feature on and off
 *
 * LCD
 * Brightness control
 * Automatic brightness contro on and off
 *
 * Camera
 * Camera on or off
 *
 */

bool checkAlarm(const char* rxString)
{
	return !strncmp(rxString,strAlarm[1],size_str);
}


bool checkFanON(const char* rxString)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	return !strncmp(rxString,strFan[1],size_str);
}
bool checkFanOFF(const char* rxString)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	return !strncmp(rxString,strFan[2],size_str);
}

bool checkLcdON(const char* rxString)
{
	return !strncmp(rxString,strLcd[1],size_str);
}
bool checkLcdOFF(const char* rxString)
{
	return !strncmp(rxString,strLcd[2],size_str);
}

uint8_t buildSummary(bool FAN_ON, bool LCD_ON, char* strConfig)
{
	int size = 0;
	uint8_t strInit[100]= "";
	memcpy(strConfig,strInit,100);
	if (LCD_ON){
//		snprintf(prefix, sizeof(prefix), "%s: %s \n", strLcd[1], strTrue);
		strcat(strConfig, strLcd[1]);
		strcat(strConfig, ": ");
		strcat(strConfig, strTrue);
		size += sizeof(strLcd[1]) + sizeof(strTrue);
	}
	else {
//					snprintf(prefix, sizeof(prefix), "%s: %s \n", strLcd[1], strFalse);
		strcat(strConfig, strLcd[1]);
		strcat(strConfig, ": ");
		strcat(strConfig, strFalse);
		size += sizeof(strLcd[1]) + sizeof(strFalse);
	}
	strcat(strConfig, "\n");

	if (FAN_ON){
//					snprintf(prefix, sizeof(prefix), "%s: %s \n", strFan[1], strTrue);
		strcat(strConfig, strFan[1]);
		strcat(strConfig, ": ");
		strcat(strConfig, strTrue);
		size += sizeof(strFan[1]) + sizeof(strTrue);
	}
	else {
//					snprintf(prefix, sizeof(prefix), "%s: %s \n", strFan[1], strFalse);
		strcat(strConfig, strFan[1]);
		strcat(strConfig, ": ");
		strcat(strConfig, strFalse);
		size += sizeof(strFan[1]) + sizeof(strFalse);
	}
	strcat(strConfig, "\n");
	return size;
}

bool checkSummary(const char* rxString)
{
	return !strncmp(rxString, strUtility[0], size_str);
}


