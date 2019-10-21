/**
* @file control.hpp
*/
#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "geometry_msgs/Twist.h"
#include "pid.hpp"


class Control
{
public:
    static void Init();
    static void SetSpeed(const geometry_msgs::Twist&);
    static PidRegulator LeftSpeed;
    static PidRegulator RightSpeed;

    static geometry_msgs::Twist GetSpeed();
};

#endif /* CONTROL_HPP */
