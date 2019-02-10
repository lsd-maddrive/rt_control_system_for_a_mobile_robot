/**
* @file encoder.hpp
* @brief Motors class
*/
#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <ch.h>
#include <hal.h>


/**
* @brief Encoder
*/
class Encoder
{
public:
    static void Init();
    static void Reset();
    static int32_t GetLeftValue();
    static int32_t GetRightValue();
    static int32_t GetLeftSpeed();
    static int32_t GetRightSpeed();
private:
};

#endif /* ENCODER_HPP */
