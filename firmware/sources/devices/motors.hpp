/**
* @file motors.hpp
* @brief Motors class
*/
#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <ch.h>
#include <hal.h>


/**
* @brief Motors
*/
class Motors
{
public:
    static void Init();
    static void SetLeftPower(int8_t power);
    static void SetRightPower(int8_t power);
private:

};

#endif /* MOTORS_HPP */
