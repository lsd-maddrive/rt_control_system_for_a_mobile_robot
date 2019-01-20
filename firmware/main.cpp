#include <ch.h>
#include <hal.h>

#include <chprintf.h>
#include "leds.hpp"
#include "serial.hpp"


int main(void)
{
    chSysInit();
    halInit();

    Led::Init();
    Led::StartBlink();

    Serial serial;
    serial.Init();

    while (true)
    {
        serial.Do();
    }
}
