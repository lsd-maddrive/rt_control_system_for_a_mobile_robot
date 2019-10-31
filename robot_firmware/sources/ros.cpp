/**
* @file ros.cpp
* @brief ros driver implementation
*/

#include "ros.hpp"
#include "encoder.hpp"
#include "leds.hpp"
#include "motors.hpp"
#include "control.hpp"
#include "odometry.hpp"
#include "debug.hpp"
#include "usb.hpp"

#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Twist.h"

#include <ch.h>
#include <hal.h>


// Bind ROS and Serial Driver
// It must be global variables!
// Must be named <ros_sd_ptr> for ChibiOSHardware
#if HAL_USE_USB == TRUE
    BaseChannel     *ros_sd_ptr = (BaseChannel *)&SDU1;
#elif HAL_USE_SERIAL_USB == TRUE
    static const SerialConfig sdcfg =
    {
         .speed = 115200,
         .cr1 = 0,
         .cr2 = USART_CR2_LINEN,
         .cr3 = 0
    };
    SerialDriver* ros_sd     = &SD5;
    BaseChannel* ros_sd_ptr = (BaseChannel*)ros_sd;
    SerialDriver    *ros_sd     = &SD5;
#endif


// Callback's
void modeSelectionCallback( const std_msgs::UInt8& msg );
void cmdCallback( const geometry_msgs::Twist& msg );

// Ros things:
// 1. Node is a process that performs computation.
static ros::NodeHandle RosNode;
// 2. Nodes communicate with each other by publishing messages to topics.
static std_msgs::Int32 EncoderLeftMsg;
static std_msgs::Int32 EncoderRightMsg;
static std_msgs::Float32 EncoderLeftSpeedMsg;
static std_msgs::Float32 EncoderRightSpeedMsg;
static std_msgs::Int32 MotorLeftMsg;
static std_msgs::Int32 MotorRightMsg;
static geometry_msgs::Point32 PositionMsg;
static geometry_msgs::Twist CmdRepeaterMsg;
static geometry_msgs::Twist SpeedMsg;
// 3. Topics are named buses over which nodes exchange messages.
static ros::Publisher EncoderLeftTopic("encoderLeftTopic", &EncoderLeftMsg);
static ros::Publisher EncoderRightTopic("encoderRightTopic", &EncoderRightMsg);
static ros::Publisher EncoderLeftSpeedTopic("encoderLeftSpeedTopic", &EncoderLeftSpeedMsg);
static ros::Publisher EncoderRightSpeedTopic("encoderRightSpeedTopic", &EncoderRightSpeedMsg);
static ros::Publisher MotorLeftTopic("motorLeftTopic", &MotorLeftMsg);
static ros::Publisher MotorRightTopic("motorRightTopic", &MotorRightMsg);
static ros::Publisher PositionTopic("positionTopic", &PositionMsg);
static ros::Publisher CmdRepeaterTopic("cmdRepeaterTopic", &CmdRepeaterMsg);
static ros::Publisher SpeedTopic("speedTopic", &SpeedMsg);
// 4. Subscribers topics:
ros::Subscriber<std_msgs::UInt8> ModeSelectionTopic("modeSelectionTopic", &modeSelectionCallback);
ros::Subscriber<geometry_msgs::Twist> CmdTopic("cmd_vel", &cmdCallback);


void modeSelectionCallback( const std_msgs::UInt8& msg )
{
    if(msg.data > 0)
        Debug::StartMovementSimulation();
    else
        Debug::StopMovementSimulation();
        
}


void cmdCallback(const geometry_msgs::Twist& msg)
{
	CmdRepeaterMsg = msg;
    Control::SetSpeed(msg);
}


// ROS thread - used to subscribe messages
static THD_WORKING_AREA(RosSubscriberThreadWorkingArea, 128);
static THD_FUNCTION(RosSubscriberThread, arg)
{
    (void)arg;
    chRegSetThreadName("RosSubscriberThread");

    while (true)
    {
        RosNode.spinOnce();
        chThdSleepMilliseconds( 10 );
    }
}


// ROS thread - used to publish messages
static THD_WORKING_AREA(RosPublisherThreadWorkingArea, 128);
static THD_FUNCTION(RosPublisherThread, arg)
{
    (void)arg;
    chRegSetThreadName("RosPublisherThread");
    
    while (true)
    {
        // Publish important system state info:
        EncoderLeftMsg.data = Encoder::GetLeftValue();
        EncoderLeftTopic.publish( &EncoderLeftMsg );

        EncoderRightMsg.data = Encoder::GetRightValue();
        EncoderRightTopic.publish( &EncoderRightMsg );

        EncoderLeftSpeedMsg.data = Encoder::GetLeftSpeed();
        EncoderLeftSpeedTopic.publish( &EncoderLeftSpeedMsg );

        EncoderRightSpeedMsg.data = Encoder::GetRightSpeed();
        EncoderRightSpeedTopic.publish( &EncoderRightSpeedMsg );

        MotorLeftMsg.data = Motors::GetLeftPower();
        MotorLeftTopic.publish( &MotorLeftMsg );

        MotorRightMsg.data = Motors::GetRightPower();
        MotorRightTopic.publish( &MotorRightMsg );

        SpeedMsg = Control::GetSpeed();
        SpeedTopic.publish(&SpeedMsg);

        OdometryPosition_t* position = Odometry::GetPosition();
        PositionMsg.x = position->x;
        PositionMsg.y = position->y;
        PositionMsg.z = position->dir;
        PositionTopic.publish(&PositionMsg);

        // Publish info for debugging:
        CmdRepeaterTopic.publish(&CmdRepeaterMsg);
        
        // Led
        static bool isLedOn = false;
        if(isLedOn)
        {
        	isLedOn = false;
        	Leds::OffThird();
        }
        else
        {
        	isLedOn = true;
        	Leds::OnThird();
        }


        chThdSleepMilliseconds(100);
    }
}


void RosDriver::Init()
{
    #if HAL_USE_USB == TRUE
        sduObjectInit( &SDU1 );
        sduStart( &SDU1, &serusbcfg );
        /*
        * Activates the USB driver and then the USB bus pull-up on D+.
        * Note, a delay is inserted in order to not have to disconnect the cable
        * after a reset.
        */
        usbDisconnectBus( serusbcfg.usbp );
        chThdSleepMilliseconds( 1500 );
        usbStart( serusbcfg.usbp, &usbcfg );
        usbConnectBus( serusbcfg.usbp );
    #elif HAL_USE_SERIAL_USB == TRUE
        sdStart( &SD5, &sdcfg );
        palSetPadMode( GPIOC, 12, PAL_MODE_ALTERNATE(8) );  // TX
        palSetPadMode( GPIOD, 2, PAL_MODE_ALTERNATE(8) );   // RX
    #endif
    
    RosNode.initNode();
    RosNode.setSpinTimeout( 20 );
    
    RosNode.advertise(EncoderLeftTopic);
    RosNode.advertise(EncoderRightTopic);
    RosNode.advertise(EncoderLeftSpeedTopic);
    RosNode.advertise(EncoderRightSpeedTopic);
    RosNode.advertise(MotorLeftTopic);
    RosNode.advertise(MotorRightTopic);
    RosNode.advertise(PositionTopic);
    RosNode.advertise(CmdRepeaterTopic);
    RosNode.advertise(SpeedTopic);

    RosNode.subscribe(CmdTopic);
    RosNode.subscribe(ModeSelectionTopic);

    chThdCreateStatic(RosSubscriberThreadWorkingArea, sizeof(RosSubscriberThreadWorkingArea), NORMALPRIO, RosSubscriberThread, NULL);
    chThdCreateStatic(RosPublisherThreadWorkingArea, sizeof(RosPublisherThreadWorkingArea), NORMALPRIO, RosPublisherThread, NULL);
}
