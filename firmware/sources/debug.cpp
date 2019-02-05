/**
* @file debug.cpp
* @brief debug implemetntation
*/

#include "debug.hpp"
#include "serial.hpp"
#include "text.hpp"
#include "adc.hpp"


/**
* @brief Work areas
* @param[in] first  - the name to be assigned to the stack array
* @param[in] second - the stack size to be assigned to the thread
*/
static THD_WORKING_AREA(debugThreadWorkingArea, 128);


/**
* @brief Debug thread function
* @param[in] DebugThread - function name
* @param[in] arg - arguments
*/
static THD_FUNCTION(DebugThread, arg)
{
    arg = arg;

    Serial* debug = Serial::GetInstance(Serial::Serial_6);

    Adc::Start();

    while (TRUE)
    {
        {
            uint32_t num = 0;
            uint8_t str[15] = {0};
            num = chVTGetSystemTime();
            num2str(num, str);
            debug->Transmit(reinterpret_cast<const uint8_t*>(str), 15);
            debug->Transmit(reinterpret_cast<const uint8_t*>("  "), 2);
        }

        for(uint8_t i = 0; i < 4; i++)
        {
            uint32_t num = 0;
            uint8_t str[15] = {0};

            num = Adc::Buffer[i];
            num2str(num, str);
            debug->Transmit(reinterpret_cast<const uint8_t*>(str), 15);
            debug->Transmit(reinterpret_cast<const uint8_t*>("  "), 2);
        }

        debug->Transmit(reinterpret_cast<const uint8_t*>("\n\r"), 2);
        chThdSleepMilliseconds(3000);
    }
}

void Debug::StartShowingSystemInfo()
{
    chThdCreateStatic(debugThreadWorkingArea, sizeof(debugThreadWorkingArea), NORMALPRIO, DebugThread, NULL);

}


