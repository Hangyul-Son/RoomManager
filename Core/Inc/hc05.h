#ifndef __HC05_H
#define __HC05_H

#include "stm32f1xx_hal.h"
#include "utility.h"

static const char strAlarm[4][8] = {"ALARM", "ALA__ON","ALA_OFF","ALA_CON"};
static const char strFan[3][8] = {"FAN", "FAN__ON", "FAN_OFF"};
static const char strLcd[3][8] = {"LCD", "LCD_ON", "LCD_OFF"};
static const char strCamera[3][8] = {"CAMERA"};
static const char strUtility[1][8] = {"STM_CON"};
static const char strFalse[] = "False";
static const char strTrue[] = "True";

bool checkAlarm(const char* rxString);
bool checkFanON(const char* rxString);
bool checkFanOFF(const char* rxString);
bool checkLcdON(const char* rxString);
bool checkLcdOFF(const char* rxString);
uint8_t buildSummary(bool FAN_ON, bool LCD_ON, char* strConfig);
bool checkSummary(const char* rxString);


#endif
