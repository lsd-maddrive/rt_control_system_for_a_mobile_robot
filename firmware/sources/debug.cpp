/**
* @file debug.cpp
* @brief debug implementation
*/

#include "debug.hpp"
#include "encoder.hpp"
#include "motors.hpp"
#include "serial.hpp"
#include "text.hpp"
#include "adc.hpp"

static thread_t* MovemetSimulationThread = nullptr;

/**
* @brief Set left encoder value 
* @note This function is private because set encoder values operations were
*       created only for debug. Methods Debug::SetEncodersValue are necessary
*       because friend is not inherited, so thread function can't use methods 
*       like Encoder::SetEncodersValue.
* @param[in] encoderValue - desired value of encoder value
*/
void Debug::SetLeftEncoderValue(int32_t encoderValue)
{
    Encoder::SetLeftValue(encoderValue);
}


/**
* @brief Set right encoder value 
* @note This function is private because set encoder values operations were
*       created only for debug. Methods Debug::SetEncodersValue are necessary
*       because friend is not inherited, so thread function can't use methods 
*       like Encoder::SetEncodersValue.
* @param[in] encoderValue - desired value of encoder value
*/
void Debug::SetRightEncoderValue(int32_t encoderValue)
{
    Encoder::SetRightValue(encoderValue);
}


/**
* @brief Work areas
* @param[in] first  - the name to be assigned to the stack array
* @param[in] second - the stack size to be assigned to the thread
*/
static THD_WORKING_AREA(debugThreadWorkingArea, 128);
static THD_WORKING_AREA(movementSimulationThreadWorkingArea, 128);


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


/**
* @brief Debug thread function
* @param[in] DebugThread - function name
* @param[in] arg - arguments
*/
THD_FUNCTION(MovementSimulationThread, arg)
{
    arg = arg;

    while (TRUE)
    {
        Debug::SetLeftEncoderValue(Encoder::GetLeftValue() + (Motors::GetLeftPower() >> 2));
        Debug::SetRightEncoderValue(Encoder::GetRightValue() + (Motors::GetRightPower() >> 2));
        
        chThdSleepMilliseconds(250);
    }
}


/**
* @brief Start thread that show system time through serial 6 directly every 3 seconds
*/
void Debug::StartShowingSystemInfoDirectly()
{
    chThdCreateStatic(debugThreadWorkingArea, sizeof(debugThreadWorkingArea), NORMALPRIO, DebugThread, NULL);

}


/**
* @brief Start thread that change values of encoders counters according motors power
*/
void Debug::StartMovementSimulation()
{
    if(MovemetSimulationThread == nullptr)
    {
        MovemetSimulationThread = chThdCreateStatic(movementSimulationThreadWorkingArea, 
                                                    sizeof(movementSimulationThreadWorkingArea), 
                                                    NORMALPRIO, 
                                                    MovementSimulationThread, 
                                                    NULL);
    }
    else
    {
        chThdStart(MovemetSimulationThread);
    }
    
}


/**
* @brief Stop thread that change values of encoders counters according motors power
*/
void Debug::StopMovementSimulation()
{
    if(MovemetSimulationThread != nullptr)
    {
        chThdWait(MovemetSimulationThread);
    }
}
