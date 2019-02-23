#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "ros.hpp"
#include "leds.hpp"
#include "encoder.hpp"


int main()
{
    chSysInit();
    halInit();
    Leds::Init();
    RosDriver::Init();
    Encoder::Init();

    while(1)
    {
        static int32_t leftValue = 0;
        static int32_t rightValue = 0;
        leftValue++;
        rightValue += 3;
        Encoder::SetLeftValue(leftValue);
        Encoder::SetRightValue(rightValue);

        chThdSleepMilliseconds(200);
    }
}
