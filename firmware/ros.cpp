/**
* @file ros.cpp
* @brief ros driver implementation
*/

#include "ros.hpp"
#include "encoder.hpp"
#include "leds.hpp"


#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Twist.h"

#include <ch.h>
#include <hal.h>
#include <chprintf.h>


// Bind ROS and Serial Driver
SerialDriver* ros_sd     = &SD5;
BaseChannel* ros_sd_ptr = (BaseChannel *)ros_sd;

// Callback's
void ledPowerCallback( const std_msgs::UInt8& msg )
{
    if(msg.data & 1)
        Leds::OnFirst();
    else
        Leds::OffFirst();

    if(msg.data & 2)
        Leds::OnSecond();
    else
        Leds::OffSecond();

    if(msg.data & 4)
        Leds::OnThird();
    else
        Leds::OffThird();
}



// Ros things:
// 1. Node is a process that performs computation.
static ros::NodeHandle RosNode;
// 2. Nodes communicate with each other by publishing messages to topics.
static std_msgs::String TestStringMsg;
static std_msgs::Int32 EncoderLeftMsg;
static std_msgs::Int32 EncoderRightMsg;
static std_msgs::Float32 EncoderLeftSpeedMsg;
static std_msgs::Float32 EncoderRightSpeedMsg;
static geometry_msgs::Point32 PositionMsg;
static geometry_msgs::Twist TurtleMsg;
// 3. Topics are named buses over which nodes exchange messages.
static ros::Publisher TestTopic("testTopic", &TestStringMsg);
static ros::Publisher EncoderLeftTopic("encoderLeftTopic", &EncoderLeftMsg);
static ros::Publisher EncoderRightTopic("encoderRightTopic", &EncoderRightMsg);
static ros::Publisher EncoderLeftSpeedTopic("encoderLeftSpeedTopic", &EncoderLeftSpeedMsg);
static ros::Publisher EncoderRightSpeedTopic("encoderRightSpeedTopic", &EncoderRightSpeedMsg);
static ros::Publisher PositionTopic("PositionTopic", &PositionMsg);
static ros::Publisher TurtleTopic("turtle1/cmd_vel", &TurtleMsg);

ros::Subscriber<std_msgs::UInt8> LedPowerTopic("LedPowerTopic", &ledPowerCallback);


// ROS thread - use to publish messages
static THD_WORKING_AREA(RosPublisherThreadWorkingArea, 128);
static THD_FUNCTION(RosPublisherThread, arg)
{
    (void)arg;
    chRegSetThreadName("RosPublisherThread");

    while (true)
    {
        TestStringMsg.data = "hello";
        TestTopic.publish( &TestStringMsg );

        EncoderLeftMsg.data = Encoder::GetLeftValue();
        EncoderLeftTopic.publish( &EncoderLeftMsg );

        EncoderRightMsg.data = Encoder::GetRightValue();
        EncoderRightTopic.publish( &EncoderRightMsg );

        EncoderLeftSpeedMsg.data = Encoder::GetLeftSpeed();
        EncoderLeftSpeedTopic.publish( &EncoderLeftSpeedMsg );

        EncoderRightSpeedMsg.data = Encoder::GetRightSpeed();
        EncoderRightSpeedTopic.publish( &EncoderRightSpeedMsg );

        TurtleMsg.linear.x = 2;
        TurtleMsg.angular.z = 2;
        TurtleTopic.publish( &TurtleMsg );

        RosNode.spinOnce();
        chThdSleepMilliseconds(1000);
    }
}


void RosDriver::Init()
{
    RosNode.initNode();
    RosNode.setSpinTimeout( 20 );
    RosNode.advertise(TestTopic);
    RosNode.advertise(EncoderLeftTopic);
    RosNode.advertise(EncoderRightTopic);
    RosNode.advertise(EncoderLeftSpeedTopic);
    RosNode.advertise(EncoderRightSpeedTopic);
    RosNode.advertise(TurtleTopic);

    RosNode.subscribe(LedPowerTopic);

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

    chThdCreateStatic(RosPublisherThreadWorkingArea, sizeof(RosPublisherThreadWorkingArea), NORMALPRIO, RosPublisherThread, NULL);
}
