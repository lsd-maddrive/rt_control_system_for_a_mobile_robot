/**
* @file odometry.cpp
* @brief Odometry implementation
*/

#include "odometry.hpp"
#include "encoder.hpp"
#include <math.h>

int32_t enc_right_cache = 0;
int32_t enc_left_cache = 0;

OdometryPosition_t pose;

const float ticks_per_rotation = 300;
const float meters_per_rotation = 0.155;
const float meters_per_tick = 0.0005167;
const float wheeltrack = 0.23;

void Odometry::Init()
{
	Reset();
}

void Odometry::Reset()
{
	pose.x = 0;
	pose.y = 0;
	pose.dir = 0;

	// encoder reset
}

OdometryPosition_t* Odometry::GetPosition()
{
	int32_t enc_left_ticks = Encoder::GetLeftValue();
	int32_t enc_right_ticks = Encoder::GetRightValue();

	float passed_path_right = (enc_left_ticks - enc_left_cache) * meters_per_tick;
	float passed_path_left = (enc_right_ticks - enc_right_cache) * meters_per_tick;

	enc_right_cache = enc_right_ticks;
	enc_left_cache = enc_left_ticks;	

	float full_path = (passed_path_right + passed_path_left) / 2;
	pose.dir = pose.dir + (passed_path_left - passed_path_right) / wheeltrack;
	pose.x = pose.x + full_path * cos( pose.dir );
	pose.y = pose.y + full_path * sin( pose.dir );

	return &pose;
}
