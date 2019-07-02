#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "ros.hpp"
#include "leds.hpp"
#include "motors.hpp"
#include "encoder.hpp"
#include "odometry.hpp"
#include "debug.hpp"


int main()
{
    chSysInit();
    halInit();
    Leds::Init();
    Motors::Init();
    RosDriver::Init();
    Encoder::Init();
    Odometry::Init();
    Debug::StartMovementSimulation();

    while(1)
    {
        chThdSleepMilliseconds(200);
    }
}
