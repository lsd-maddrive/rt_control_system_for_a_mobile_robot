#include <ch.h>
#include <hal.h>

#include <chprintf.h>
#include "leds.hpp"
#include "serial.hpp"
#include "adc.hpp"
#include "pwm.hpp"
#include "debug.hpp"

int main(void)
{
    chSysInit();
    halInit();

    //Led::Init();
    //Led::StartBlink();

    Pwm led1, led2;
    led1.Create(Pwm::PIN_PB0);
    led2.Create(Pwm::PIN_PB7);

    Debug::StartShowingSystemInfo();
    while (true)
    {

        for(int8_t dc = 0; dc <= 50; dc += 10)
        {
            led1.Start(dc);
            led2.Start(50 - dc);
            chThdSleepMilliseconds(50);
        }
        for(int8_t dc = 50; dc >= 0; dc -= 1)
        {
            led1.Start(dc);
            led2.Start(50 - dc);
            chThdSleepMilliseconds(50);
        }

    }
}
