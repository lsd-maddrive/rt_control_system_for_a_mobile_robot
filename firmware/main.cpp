#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "ros.hpp"
#include "leds.hpp"


int main()
{
    chSysInit();
    halInit();
    Leds::Init();
    RosDriver::Init();

    while(1)
    {
        chThdSleepMilliseconds(100);
    }
}


/*
int main(void)
{
    chSysInit();
    halInit();

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
*/
