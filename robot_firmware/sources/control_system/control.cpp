/**
* @file motors.cpp
*/

#include <ch.h>

#include "control.hpp"
#include "motors.hpp"
#include "encoder.hpp"
#include "odometry.hpp"

static thread_t* PidProcessThreadPointer = nullptr;
static THD_WORKING_AREA(pidProcessThreadWorkingArea, 128);

PidRegulator Control::RightSpeed(600, 600, 0);
PidRegulator Control::LeftSpeed(600, 600, 0);

/**
* @brief Pid regulator process thread function
* @param[in] PidProcessThread - function name
* @param[in] arg - arguments
*/
THD_FUNCTION(PidProcessThread, arg)
{
    arg = arg;

    const float meters_per_tick = 0.0005167;

    while (TRUE)
    {
    	float currentLeftSpeed = Encoder::GetLeftSpeed() * meters_per_tick;
    	float currentRightSpeed = Encoder::GetLeftSpeed() * meters_per_tick;
        Motors::SetLeftPower(Control::LeftSpeed.Do(currentLeftSpeed));
        Motors::SetRightPower(Control::RightSpeed.Do(currentRightSpeed));

        chThdSleepMilliseconds(25);
    }
}

void Control::Init()
{
	if(PidProcessThreadPointer == nullptr)
	{
		PidProcessThreadPointer = chThdCreateStatic(pidProcessThreadWorkingArea,
	                              sizeof(pidProcessThreadWorkingArea),
	                              NORMALPRIO,
								  PidProcessThread,
	                              NULL);
	}
	LeftSpeed.SetValue(0);
	RightSpeed.SetValue(0);
}

void Control::SetSpeed(const geometry_msgs::Twist& msg)
{
    float linear = msg.linear.x;
    float rotation = msg.angular.z;

    if(linear)
    {
    	LeftSpeed.SetValue(linear);
    	RightSpeed.SetValue(linear);
    }
    else if(rotation)
    {
        Motors::SetLeftPower(-100 * rotation);
        Motors::SetRightPower(100 * rotation);
    }
    else
    {
        Motors::SetLeftPower(0);
        Motors::SetRightPower(0);
    }
}
