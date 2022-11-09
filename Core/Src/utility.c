
#include "utility.h"

void DelayMicroSeconds(uint32_t uSec)
{
    uint32_t uSecVar = uSec;
    uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
    while(uSecVar--);
}
