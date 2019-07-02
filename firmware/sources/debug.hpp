/**
* @file debug.hpp
* @brief Debug driver class
*/
#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <ch.h>


/**
* @brief Debug
* @details It contains some methods for debugging the board through serial 
* directly or by using ros communication protocol
*/
class Debug
{
public:
    /// @brief Show system time through serial 6 directly every 3 seconds
    static void StartShowingSystemInfoDirectly();
    /// @brief Change values of encoders counters according motor power
    static void StartMovementSimulation();
private:
    friend void MovementSimulationThread(void* arg);
    static void SetLeftEncoderValue(int32_t encoderValue);
    static void SetRightEncoderValue(int32_t encoderValue);
};

#endif /* DEBUG_HPP */
