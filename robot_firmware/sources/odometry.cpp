/**
* @file odometry.cpp
* @brief Odometry implementation
*/

#include <math.h>
#include <stdint.h>
#include "odometry.hpp"
#include "encoder.hpp"
#include "robot_calibration.hpp"

static int32_t EncRightCache = 0;
static int32_t EncLeftCache = 0;
static OdometryPosition_t pose;

void Odometry::Init()
{
	Reset();
}


void Odometry::Reset()
{
	pose.x = 0;
	pose.y = 0;
	pose.dir = 0;
}


OdometryPosition_t* Odometry::GetPosition()
{
	int32_t encLeftTicks = Encoder::GetLeftValue();
	int32_t encRightTicks = Encoder::GetRightValue();

	float passedPathLeft = (encLeftTicks - EncLeftCache) * METERS_PER_TICK;
	float passedPathRight = (encRightTicks - EncRightCache) * METERS_PER_TICK;

	EncRightCache = encRightTicks;
	EncLeftCache = encLeftTicks;

	float fullPath = (passedPathRight + passedPathLeft) / 2;
	pose.dir = pose.dir + (passedPathLeft - passedPathRight) / WHEELTRACK;
	pose.x = pose.x + fullPath * cos( pose.dir );
	pose.y = pose.y + fullPath * sin( pose.dir );

	return &pose;
}
