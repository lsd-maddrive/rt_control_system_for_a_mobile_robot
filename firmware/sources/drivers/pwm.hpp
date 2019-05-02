/**
* @file pwm.hpp
* @brief Pwm driver class
*/
#ifndef PWM_HPP
#define PWM_HPP

#include <hal.h>

/**
* @brief Pwm driver
*/
class Pwm
{
public:
    static void Init();
    static void MotorLeftSetDutyCycle(int8_t dutyCycle);
    static void MotorRightSetDutyCycle(int8_t dutyCycle);
    static int8_t MotorLeftGetDutyCycle();
    static int8_t MotorRightGetDutyCycle();
};

#endif /* PWM_HPP */
