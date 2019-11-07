#!/usr/bin/env python
import rospy
import json
import matplotlib.pyplot as plt
import numpy
import os
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist, Vector3, Point32

# Choose mode
# 1. Test mode
ENABLE_TEST = True
TIME_BEFORE_SPEED_CHANGE = float(2)
DESIRED_LINEAR_SPEEDS =  [+0.15, -0.15, +0.00, +0.00, +0.00, +0.00]
DESIRED_ANGULAR_SPEEDS = [+0.00, -0.00, +0.00, +0.50, -0.50, +0.00]
# 2. Real mode
DATA_GATHERING_TIME = 30
# 3. Other settings
REAL_TIME_PLOTTING = False


# Constants
CMD_TOPIC = "/cmd_vel"
SPEED_TOPIC = "/speedTopic"
ENCODER_LEFT_SPEED_TOPIC = "/encoderLeftSpeedTopic"
ENCODER_RIGHT_SPEED_TOPIC = "/encoderRightSpeedTopic"
LEFT_MOTOR_TOPIC = "/motorLeftTopic"
RIGHT_MOTOR_TOPIC = "/motorRightTopic"
POSITION_TOPIC = "/positionTopic"

THIS_NODE_NAME = 'test_pid_node'
JSON_FILE_NAME = "test_pid_result.txt"

rospy.init_node(THIS_NODE_NAME)

class Topic:
    def initCommon(self, topic):
        self.TOPIC = topic
        self.time = list()
        self.sub = rospy.Subscriber(self.TOPIC, self.dataType, self.handle)
        self.__startTime = rospy.get_rostime()
    def appendRelativeTime(self):
        currentTime = rospy.get_rostime()
        relativeSecs = currentTime.secs - self.__startTime.secs
        relativeNsecs = currentTime.nsecs - self.__startTime.nsecs
        relativeTime = relativeSecs + float(relativeNsecs)/1000000000
        self.time.append(relativeTime)
    def off(self):
        self.sub.unregister()
    def getDictData(self):
        return dict([("time", self.time), (self.TOPIC, self.data)])

class Speed(Topic):
    def __init__(self, topic):
        self.linear = list()
        self.angular = list()
        self.dataType = Twist
        self.initCommon(topic)
    def handle(self, msg):
        self.appendRelativeTime()
        print("received: {0}, {1}, {2}".format(self.TOPIC, self.time[-1], msg.linear.x))
        print("received: {0}, {1}, {2}".format(self.TOPIC, self.time[-1], msg.angular.z))
        self.linear.append(msg.linear.x)
        self.angular.append(msg.angular.z)
    def createPlot(self, speedType, desiredSpeeds):
        # Create desirable plot
        if ENABLE_TEST is True:
            if speedType is "linear":
                desiredSpeeds = DESIRED_LINEAR_SPEEDS
            elif speedType is "angular":
                desiredSpeeds = DESIRED_ANGULAR_SPEEDS
            referenceTime = list([0, 0])
            referenceSpeed = list([0, desiredSpeeds[0]])
            for i in range(1, len(desiredSpeeds)):
                referenceTime.append(i * TIME_BEFORE_SPEED_CHANGE)
                referenceTime.append(i * TIME_BEFORE_SPEED_CHANGE)
                referenceSpeed.append(desiredSpeeds[i - 1])
                referenceSpeed.append(desiredSpeeds[i])
            referenceTime.append( TIME_BEFORE_SPEED_CHANGE * len(desiredSpeeds) )
            referenceSpeed.append(desiredSpeeds[-1])
            plt.plot(referenceTime, referenceSpeed)
        # Crete real plot
        if speedType is "linear":
            plt.plot(self.time, self.linear)
            plt.title('Linear speed')
            if len(self.time) is 0:
                plt.axis([0, 0.01, -0.25, 0.25])
            else:
                plt.axis([self.time[0], self.time[-1], -0.25, 0.25])
        elif speedType is "angular":
            plt.plot(self.time, self.angular)
            plt.title('Angular speed')
            if len(self.time) is 0:
                plt.axis([0, 0.01, -0.25, 0.25])
            else:
                plt.axis([self.time[0], self.time[-1] + 0.01, -1, 1])
        plt.grid()
    def getDictData(self, dataType):
        if dataType == "linear":
            return dict([("time", self.time), (self.TOPIC, self.linear)])
        elif dataType == "angular":
            return dict([("time", self.time), (self.TOPIC, self.angular)])
        else:
            return None

class EncoderSpeed(Topic):
    def __init__(self, topic):
        self.data = list()
        self.dataType = Float32
        self.initCommon(topic)
    def handle(self, msg):
        self.appendRelativeTime()
        print("received: {0}, {1}, {2}".format(self.TOPIC, self.time[-1], msg.data))
        self.data.append(msg.data)
    def createPlot(self):
        plt.plot(self.time, self.data)
        plt.title('Encoder speed')
        plt.grid()


class MotorSpeed(Topic):
    def __init__(self, topic):
        self.data = list()
        self.dataType = Int32
        self.initCommon(topic)
    def handle(self, msg):
        self.appendRelativeTime()
        print("received: {0}, {1}, {2}".format(self.TOPIC, self.time[-1], msg.data))
        self.data.append(msg.data)
    def createPlot(self):
        plt.plot(self.time, self.data)
        plt.title('Motor duty cycle')
        plt.grid()


class Position(Topic):
    def __init__(self, topic):
        self.dataType = Point32
        self.x = list()
        self.y = list()
        self.z = list()
        self.initCommon(topic)
    def handle(self, msg):
        self.appendRelativeTime()
        print("received: {0}, {1}, {2}".format(self.TOPIC, self.time[-1], 
                                               msg.x, msg.y, msg.z))
        self.x.append(msg.x)
        self.y.append(msg.y)
        self.z.append(msg.z)
    def createPlot(self, dataType):
        if dataType == "x":
            plt.plot(self.time, self.x)
            plt.ylabel('x, meters')
            plt.title('Position x')
        elif dataType == "y":
            plt.plot(self.time, self.y)
            plt.ylabel('y, meters')
            plt.title('Position y')
        elif dataType == "z":
            plt.plot(self.time, self.z)
            plt.title('Direction')
        plt.xlabel('t, sec')
        plt.grid()
    def getDictData(self, dataType):
        if dataType is "x":
            return dict([("time", self.time), (self.TOPIC, self.x)])
        elif dataType is "y":
            return dict([("time", self.time), (self.TOPIC, self.y)])
        elif dataType is "z":
            return dict([("time", self.time), (self.TOPIC, self.z)])
        else:
            return None


def start_data_collection():
    # Initialization
    print("Node have started. Sleep for short time...")
    print("The settings are:")
    if ENABLE_TEST is True:
        print("- test enabled with changing speed every " + \
              str(TIME_BEFORE_SPEED_CHANGE) + " secs with following speeds.")
    else:
        print("- gathering real data for" + str(DATA_GATHERING_TIME) + \
              "seconds")

    cmd = rospy.Publisher(CMD_TOPIC, Twist, queue_size = 10)
    rospy.sleep(0.1)
    msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    cmd.publish(msg)
    rate = rospy.Rate(1 / TIME_BEFORE_SPEED_CHANGE)
    rate.sleep()

    # Gather data
    print("Data is gathering now...")
    speed = Speed(SPEED_TOPIC)
    encLeftSpeed = EncoderSpeed(ENCODER_LEFT_SPEED_TOPIC)
    encRightSpeed = EncoderSpeed(ENCODER_RIGHT_SPEED_TOPIC)
    leftMotorSpeed = MotorSpeed(LEFT_MOTOR_TOPIC)
    rightMotorSpeed = MotorSpeed(RIGHT_MOTOR_TOPIC)
    position = Position(POSITION_TOPIC)

    if ENABLE_TEST is True:
        for i in range(0, len(DESIRED_LINEAR_SPEEDS)):
            msg.linear.x = DESIRED_LINEAR_SPEEDS[i]
            msg.angular.z = DESIRED_ANGULAR_SPEEDS[i]
            cmd.publish(msg)
            print("\ncommand:\n" + str(msg))
            rate.sleep()
    else:
        rospy.sleep(DATA_GATHERING_TIME)

    speed.off()
    encLeftSpeed.off()
    encRightSpeed.off()
    leftMotorSpeed.off()
    rightMotorSpeed.off()
    position.off()

    # Write to file
    write_file = open(JSON_FILE_NAME, "w")
    json.dump(list([ speed.getDictData("linear"),
                     encLeftSpeed.getDictData(),
                     leftMotorSpeed.getDictData(),
                     speed.getDictData("angular"),
                     encRightSpeed.getDictData(),
                     rightMotorSpeed.getDictData(), 
                     position.getDictData("x"),
                     position.getDictData("y"),
                     position.getDictData("z")]),
              write_file, indent=2)
    print("File with name", os.getcwd() + "/" + JSON_FILE_NAME, "has just been created.")
    print("Finish.")
    
    # Create plot
    if REAL_TIME_PLOTTING is False:
        plt.figure()
        plt.subplot(3, 3, 1); speed.createPlot("linear", DESIRED_LINEAR_SPEEDS)
        plt.subplot(3, 3, 2); encLeftSpeed.createPlot()
        plt.subplot(3, 3, 3); leftMotorSpeed.createPlot()
        plt.subplot(3, 3, 4); speed.createPlot("angular", DESIRED_ANGULAR_SPEEDS)
        plt.subplot(3, 3, 5); encRightSpeed.createPlot()
        plt.subplot(3, 3, 6); rightMotorSpeed.createPlot()
        plt.subplot(3, 3, 7); position.createPlot("x")
        plt.subplot(3, 3, 8); position.createPlot("y")
        plt.subplot(3, 3, 9); position.createPlot("z")
        plt.show()


try:
    start_data_collection()
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')
