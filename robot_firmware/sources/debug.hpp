/**
* @file debug.hpp
* @brief Debug driver class
*/
#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <stdint.h>

/**
* @brief Debug
* @details It contains some methods for debugging the board through serial 
* directly or by using ros communication protocol
*/
class Debug
{
public:
    /// @brief Start thread that change values of encoders counters according motors power
    static void StartMovementSimulation();
    /// @brief Stop thread that change values of encoders counters according motors power
    static void StopMovementSimulation();
private:
    friend void MovementSimulationThread(void* arg);
    static void SetLeftEncoderValue(int32_t encoderValue);
    static void SetRightEncoderValue(int32_t encoderValue);
};

#endif /* DEBUG_HPP */
