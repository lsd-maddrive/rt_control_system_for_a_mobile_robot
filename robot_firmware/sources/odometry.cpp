/**
* @file odometry.cpp
* @brief Odometry implementation
*/

#include <math.h>
#include <stdint.h>
#include "odometry.hpp"
#include "encoder.hpp"

static int32_t enc_right_cache = 0;
static int32_t enc_left_cache = 0;

static OdometryPosition_t pose;

/// Encoder Constant
static const float ticks_per_rotation __attribute__((unused)) = 300;
/// Calibration constants
static const float meters_per_rotation __attribute__((unused)) = 0.155;
static const float meters_per_tick = 0.0005167;
static const float wheeltrack = 0.23;


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
