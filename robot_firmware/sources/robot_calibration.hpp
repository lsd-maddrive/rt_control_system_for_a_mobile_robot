/**
* @file robot_calibration.hpp
* @brief Constants which were received by calibration
*/
#ifndef ROBOT_CALIBRATION_HPP
#define ROBOT_CALIBRATION_HPP

/// Encoder Constant
static constexpr float TICKS_PER_ROTATION __attribute__((unused)) = 300;

/// Calibration constants
static constexpr float METERS_PER_ROTATION __attribute__((unused)) = 0.155;
static constexpr float METERS_PER_TICK = 0.0005167;
static constexpr float WHEELTRACK = 0.23;

/// PID constants
static constexpr float P_DEFAULT = 0.025;
static constexpr float I_DEFAULT = 0.25;
static constexpr float D_DEFAULT = 0;

// Robot Configuration Parameters
static constexpr float MAX_ABS_LINEAR_SPEED = 0.25;
static constexpr float MIN_ABS_LINEAR_SPEED = 0.05;

static constexpr float MAX_ABS_ANGULAR_SPEED = 1.57;
static constexpr float MIN_ABS_ANGULAR_SPEED = 0.05;

static constexpr float MIN_ABS_WHEEL_SPEED = MIN_ABS_LINEAR_SPEED;
static constexpr float MAX_ABS_WHEEL_SPEED = MAX_ABS_ANGULAR_SPEED;


#endif /* ROBOT_CALIBRATION_HPP */
