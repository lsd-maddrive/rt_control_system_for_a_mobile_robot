/**
* @file debug.cpp
* @brief debug implementation
*/

#include <ch.h>
#include "debug.hpp"
#include "encoder.hpp"
#include "motors.hpp"
#include "leds.hpp"


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

    static float needLeftImpulses = 0;
	static float needRightImpulses = 0;

    while (TRUE)
    {
    	needLeftImpulses += Motors::GetLeftPower() * 0.05;
    	needRightImpulses += Motors::GetRightPower() * 0.05;
        Debug::AddLeftEncoderValue(static_cast<int32_t>(needLeftImpulses));
        Debug::AddRightEncoderValue(static_cast<int32_t>(needRightImpulses));
        needLeftImpulses -= static_cast<int32_t>(needLeftImpulses);
        needRightImpulses -= static_cast<int32_t>(needRightImpulses);
        chThdSleepMilliseconds(10);
    }
}


void Debug::StartMovementSimulation()
{
    Leds::OnSecond();
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
	Leds::OffSecond();
    if(MovementSimulationThreadPointer != nullptr)
    {
        chThdWait(MovementSimulationThreadPointer);
    }
}


/**
* @brief Add impulses to left encoder
* @note This function is private because set encoder values operations were
*       created only for debug. Methods Debug::SetEncodersValue are necessary
*       because friend is not inherited, so thread function can't use methods 
*       like Encoder::SetEncodersValue.
* @param[in] encoderValue - desired value of encoder value
*/
void Debug::AddLeftEncoderValue(int32_t encoderValue)
{
	if(encoderValue != 0)
		Encoder::SetLeftValue(Encoder::GetLeftValue() + encoderValue);
}


/**
* @brief Add impulses to right encoder
* @note This function is private because set encoder values operations were
*       created only for debug. Methods Debug::SetEncodersValue are necessary
*       because friend is not inherited, so thread function can't use methods 
*       like Encoder::SetEncodersValue.
* @param[in] encoderValue - desired value of encoder value
*/
void Debug::AddRightEncoderValue(int32_t encoderValue)
{
	if(encoderValue != 0)
		Encoder::SetRightValue(Encoder::GetRightValue() + encoderValue);
}
