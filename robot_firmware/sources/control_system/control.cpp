/**
* @file control.cpp
*/

#include <ch.h>
#include <stdlib.h>

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

    	int8_t leftPower = Control::LeftSpeed.Do(currentLeftSpeed);
    	int8_t rightPower = Control::RightSpeed.Do(currentRightSpeed);

    	if (abs(Control::LeftSpeed.GetDesiredValue()) < MIN_ABS_WHEEL_SPEED)
    	{
    		leftPower = 0;
    	}
    	if (abs(Control::RightSpeed.GetDesiredValue()) < MIN_ABS_WHEEL_SPEED)
    	{
    		rightPower = 0;
    	}

    	Motors::SetLeftPower(leftPower);
    	Motors::SetRightPower(rightPower);
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
* @param msg - desired speed (linear.x, angular.z)
* @note PID regulators work with following parameters:
* - input - encoders speeds which will provide desired speed
* - output - result motor pwm
*/
void Control::SetSpeed(const geometry_msgs::Twist& msg)
{
	float linear = msg.linear.x;
	float rotation = msg.angular.z;

	float linearSpeedInTicks = linear / METERS_PER_TICK;
	float angularSpeedInTicks = rotation / METERS_PER_TICK * WHEELTRACK / 2;

	float leftSpeed;
	float rightSpeed;
	if( (abs(linear) < MIN_ABS_LINEAR_SPEED) && (abs(rotation) < MIN_ABS_ANGULAR_SPEED) )
	{
		leftSpeed = 0;
		rightSpeed = 0;
	}
	else if( (abs(linear) >= MIN_ABS_LINEAR_SPEED) && (abs(rotation) >= MIN_ABS_ANGULAR_SPEED) )
	{
		leftSpeed = linearSpeedInTicks - angularSpeedInTicks;
		rightSpeed = linearSpeedInTicks + angularSpeedInTicks;
	}
	else if( abs(linear) >= MIN_ABS_LINEAR_SPEED )
	{
		leftSpeed = linearSpeedInTicks;
		rightSpeed = linearSpeedInTicks;
	}
	else if( abs(rotation) >= MIN_ABS_ANGULAR_SPEED )
	{
		leftSpeed = -angularSpeedInTicks;
		rightSpeed = angularSpeedInTicks;
	}
	LeftSpeed.SetValue(leftSpeed);
	RightSpeed.SetValue(rightSpeed);
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
