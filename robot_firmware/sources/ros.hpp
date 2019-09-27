/**
* @file ros.hpp
* @brief Ros driver class
*/
#ifndef ROS_HPP
#define ROS_HPP

#include <ch.h>


class RosDriver
{
public:
    /**
    * @brief Init ros work
    * @details Ros work initialization consist of following actions:
    * - start serial or usb driver
    * - init ros node
    * - advertise and subscribe topics
    * - start two threads (for subscribers and publishers)
    */
    static void Init();
};


#endif /* ROS_HPP */
