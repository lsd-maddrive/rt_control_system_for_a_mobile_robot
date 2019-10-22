/**
* @file control.cpp
*/

#include <ch.h>

#include "control.hpp"
#include "motors.hpp"
#include "encoder.hpp"
#include "odometry.hpp"
#include "robot_calibration.hpp"

static constexpr float DELTA_TIME_S = 0.025;
static constexpr uint32_t DELTA_TIME_MS = DELTA_TIME_S * 1000;

static thread_t* PidProcessThreadPointer = nullptr;
static THD_WORKING_AREA(pidProcessThreadWorkingArea, 128);

PidRegulator Control::RightSpeed(P_DEFAULT, I_DEFAULT, D_DEFAULT, DELTA_TIME_S);
PidRegulator Control::LeftSpeed(P_DEFAULT, I_DEFAULT, D_DEFAULT, DELTA_TIME_S);

/**
* @brief Pid regulator process thread function
* @param[in] PidProcessThread - function name
* @param[in] arg - arguments
*/
THD_FUNCTION(PidProcessThread, arg)
{
    arg = arg;

    while (TRUE)
    {
    	float currentLeftSpeed = Encoder::GetLeftSpeed();
    	float currentRightSpeed = Encoder::GetRightSpeed();
        Motors::SetLeftPower(Control::LeftSpeed.Do(currentLeftSpeed));
        Motors::SetRightPower(Control::RightSpeed.Do(currentRightSpeed));
        chThdSleepMilliseconds(DELTA_TIME_MS);
    }
}


/**
* @brief Start thread for pid regulators
*/
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


/**
* @param msg - desired speed (linear.x, linear.y, angular.z)
* @note PID regulators work with following parameters:
* - input - encoders speeds which will provide desired speed
* - output - result motor pwm
*/
void Control::SetSpeed(const geometry_msgs::Twist& msg)
{
    float linear = msg.linear.x;
    float rotation = msg.angular.z;

    if(linear)
    {
    	LeftSpeed.SetValue(linear / METERS_PER_TICK);
    	RightSpeed.SetValue(linear / METERS_PER_TICK);
    }
    else if(rotation)
    {
        LeftSpeed.SetValue(-rotation / METERS_PER_TICK * WHEELTRACK / 2);
		RightSpeed.SetValue(rotation / METERS_PER_TICK * WHEELTRACK / 2);
    }
    else
    {
        Motors::SetLeftPower(0);
        Motors::SetRightPower(0);
        LeftSpeed.SetValue(0);
		RightSpeed.SetValue(0);
    }
}


geometry_msgs::Twist Control::GetSpeed()
{
	geometry_msgs::Twist speed;

	auto leftEncSpeed = Encoder::GetLeftSpeed();
	auto rightEncSpeed = Encoder::GetRightSpeed();

	auto leftWheelSpeed = leftEncSpeed * METERS_PER_TICK;
	auto rightWheelSpeed = rightEncSpeed * METERS_PER_TICK;

	speed.linear.x = (rightWheelSpeed + leftWheelSpeed) / 2;
	speed.angular.z = (rightWheelSpeed - leftWheelSpeed) / WHEELTRACK;

	return speed;
}
