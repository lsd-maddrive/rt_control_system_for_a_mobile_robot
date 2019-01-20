#include <ch.h>
#include <hal.h>

#include <chprintf.h>
#include "leds.hpp"

// Рабочее пространство и функция потока - мигаем светодиодом 3
static THD_WORKING_AREA(ledThirdThreadWorkingArea, 128);
static THD_FUNCTION(LedThirdThread, arg)
{
    arg = arg;
    Led::Init();
    while (TRUE)
    {
        Led::OnThird();
        chThdSleepMilliseconds(250);
        Led::OffThird();
        chThdSleepMilliseconds(250);
    }
}

// Рабочее пространство и функция потока - мигаем светодиодом 2
static THD_WORKING_AREA(ledSecondThreadWorkingArea, 128);
static THD_FUNCTION(LedSecondThread, arg)
{
    arg = arg;
    Led::Init();
    while (TRUE)
    {
        Led::OnSecond();
        chThdSleepMilliseconds(500);
        Led::OffSecond();
        chThdSleepMilliseconds(500);
    }
}

int main(void)
{
    chSysInit();
    halInit();

    chThdCreateStatic(ledThirdThreadWorkingArea, sizeof(ledThirdThreadWorkingArea), NORMALPRIO, LedThirdThread, NULL /* arg is NULL */);
    chThdCreateStatic(ledSecondThreadWorkingArea, sizeof(ledSecondThreadWorkingArea), NORMALPRIO, LedSecondThread, NULL /* arg is NULL */);
    while (true)
    {
        Led::OnFirst();
        chThdSleepSeconds(1);
        Led::OffFirst();
        chThdSleepSeconds(1);
    }
}
