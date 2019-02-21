/**
* @file ros.cpp
* @brief ros driver implementation
*/

#include <ros.hpp>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <ros.h>

#include <std_msgs/String.h>


// Bind ROS and Serial Driver
SerialDriver* ros_sd     = &SD5;
BaseChannel* ros_sd_ptr = (BaseChannel *)ros_sd;


// Ros things
ros::NodeHandle rosNode;
std_msgs::String testStringMsg;
ros::Publisher testTopic("chatter", &testStringMsg);


void RosDriver::Init()
{
    rosNode.initNode();
    rosNode.setSpinTimeout( 20 );
    rosNode.advertise(testTopic);

    const SerialConfig sdcfg =
    {
         .speed = 115200,
         .cr1 = 0,
         .cr2 = USART_CR2_LINEN,
         .cr3 = 0
    };
    sdStart( &SD5, &sdcfg );
    palSetPadMode( GPIOC, 12, PAL_MODE_ALTERNATE(8) );  // TX
    palSetPadMode( GPIOD, 2, PAL_MODE_ALTERNATE(8) );   // RX
}


void RosDriver::Publish()
{
    testStringMsg.data = "hello";
    testTopic.publish( &testStringMsg );
    rosNode.spinOnce();
    chThdSleepMilliseconds(1000);
}

