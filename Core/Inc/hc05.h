#ifndef __HC05_H
#define __HC05_H

#include "stm32f1xx_hal.h"
#include "utility.h"

static const uint8_t strAlarm[4][8] = {"ALARM", "ALA__ON","ALA_OFF","ALA_CON"};
static const uint8_t strFan[3][8] = {"FAN", "FAN__ON", "FAN_OFF"};
static const uint8_t strLcd[3][8] = {"LCD", "LCD_ON", "LCD_OFF"};
static const uint8_t strCamera[3][8] = {"CAMERA"};
static const uint8_t strUtility[1][8] = {"STM_CON"};
static const uint8_t strFalse[] = "False";
static const uint8_t strTrue[] = "True";

void checkAlarm(uint8_t* rxString);
bool checkFanON(uint8_t* rxString);
bool checkFanOFF(uint8_t* rxString);
bool checkLcdON(uint8_t* rxString);
bool checkLcdOFF(uint8_t* rxString);
uint8_t buildSummary(bool FAN_ON, bool LCD_ON, uint8_t* strConfig);
bool checkSummary(uint8_t* rxString);


#endif
