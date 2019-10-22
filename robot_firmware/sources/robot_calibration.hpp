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
static constexpr float I_DEFAULT = 1;
static constexpr float D_DEFAULT = 0;

/// PID constants


#endif /* ROBOT_CALIBRATION_HPP */
