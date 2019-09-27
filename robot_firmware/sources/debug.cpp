/**
* @file debug.cpp
* @brief debug implementation
*/

#include "debug.hpp"
#include "encoder.hpp"
#include "motors.hpp"


static thread_t* MovementSimulationThreadPointer = nullptr;
static THD_WORKING_AREA(movementSimulationThreadWorkingArea, 128);


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


void Debug::StartMovementSimulation()
{
    if(MovementSimulationThreadPointer == nullptr)
    {
        MovementSimulationThreadPointer = chThdCreateStatic(movementSimulationThreadWorkingArea, 
                                                    sizeof(movementSimulationThreadWorkingArea), 
                                                    NORMALPRIO, 
                                                    MovementSimulationThread, 
                                                    NULL);
    }
    else
    {
        chThdStart(MovementSimulationThreadPointer);
    }
    
}


void Debug::StopMovementSimulation()
{
    if(MovementSimulationThreadPointer != nullptr)
    {
        chThdWait(MovementSimulationThreadPointer);
    }
}


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
