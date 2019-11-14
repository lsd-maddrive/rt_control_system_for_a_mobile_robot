#!/usr/bin/env python
""" This script starts pid test and/or create plots for result."""
import json
import os
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist, Vector3, Point32

# Choose mode
# 1. Test mode
ENABLE_TEST = False
TIME_BEFORE_SPEED_CHANGE = float(2)
DESIRED_LINEAR_SPEEDS = [+0.15, -0.15, +0.00, +0.00, +0.00, +0.00]
DESIRED_ANGULAR_SPEEDS = [0.00, -0.00, +0.00, +0.50, -0.50, +0.00]
DESIRED_SPEED_LENGTH = len(DESIRED_LINEAR_SPEEDS)
# 2. Real mode
DATA_GATHERING_TIME = 30
# 3. Other settings
REAL_TIME_PLOTTING = False
PLOT_ONLY = True


# Constants
class TopicName:
    CMD = "/cmd_vel"
    SPEED = "/speedTopic"
    ENCODER_LEFT_SPEED = "/encoderLeftSpeedTopic"
    ENCODER_RIGHT_SPEED = "/encoderRightSpeedTopic"
    LEFT_MOTOR = "/motorLeftTopic"
    RIGHT_MOTOR = "/motorRightTopic"
    POSITION = "/positionTopic"

THIS_NODE_NAME = 'test_pid_node'
FILE_NAME_BASE = "test_pid_result_"
FILE_NAME_END = ".json"

rospy.init_node(THIS_NODE_NAME)


class Json:
    LINEAR_SPEED = "linear speed"
    LEFT_ENCODER_SPEED = "left encoder speed"
    LEFT_MOTOR_SPEED = "left motor pwm"
    ANGULAR_SPEED = "angular speed"
    RIGHT_ENCODER_SPEED = "right encoder speed"
    RIGHT_MOTOR_SPEED = "right motor pwm"
    POSITION_X = "position x"
    POSITION_Y = "position y"
    POSITION_DIR = "position dir"

    DATA = "data"
    TIME = "time"

class Topic:
    def initCommon(self, topic):
        self.TOPIC = topic
        self.time = list()
        self.sub = rospy.Subscriber(self.TOPIC, self.dataType, self.handle)
        self.__start_time = rospy.get_rostime()
    def appendRelativeTime(self):
        current_time = rospy.get_rostime()
        relative_secs = current_time.secs - self.__start_time.secs
        relative_n_secs = current_time.nsecs - self.__start_time.nsecs
        relative_time = relative_secs + float(relative_n_secs)/1000000000
        self.time.append(relative_time)
    def off(self):
        self.sub.unregister()

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
    def createPlot(self, speedType):
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
        print("received: {0}, {1}, {2}, {3}".format(self.TOPIC, self.time[-1],
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


class Test():
    def do(self, mode):
        if PLOT_ONLY is True:
            if ENABLE_TEST is True:
                self.create_desired_plot()
            self.read_data_and_create_plot("test_pid_result_1.json")
        else:
            self.sensors = list()
            self.fileName = ""
            self.set_mode_and_show_info(mode)
            self.gather_data()
            self.write_data_to_file()
            if ENABLE_TEST is True:
                self.create_desired_plot()
            self.read_data_and_create_plot()

    def set_mode_and_show_info(self, mode):
        print("Node have started. Sleep for short time...\nThe settings are:")
        if ENABLE_TEST is True:
            print("- test enabled")
            print("- speed change interval: " + str(TIME_BEFORE_SPEED_CHANGE))
        else:
            print("- gathering real data")
            print("- time interval is" + str(DATA_GATHERING_TIME) + "seconds")
    def gather_data(self):
        # Preparation
        cmd = rospy.Publisher(TopicName.CMD, Twist, queue_size=10)
        rospy.sleep(0.1)
        msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        cmd.publish(msg)
        rate = rospy.Rate(1 / TIME_BEFORE_SPEED_CHANGE)
        rate.sleep()
        # Gather data
        print("Data is gathering now...")
        self.sensors = list()
        self.sensors.append(Speed(TopicName.SPEED))
        self.sensors.append(EncoderSpeed(TopicName.ENCODER_LEFT_SPEED))
        self.sensors.append(EncoderSpeed(TopicName.ENCODER_RIGHT_SPEED))
        self.sensors.append(MotorSpeed(TopicName.LEFT_MOTOR))
        self.sensors.append(MotorSpeed(TopicName.RIGHT_MOTOR))
        self.sensors.append(Position(TopicName.POSITION))
        if ENABLE_TEST is True:
            for i in range(0, len(DESIRED_LINEAR_SPEEDS)):
                msg.linear.x = DESIRED_LINEAR_SPEEDS[i]
                msg.angular.z = DESIRED_ANGULAR_SPEEDS[i]
                cmd.publish(msg)
                print("\ncommand:\n" + str(msg))
                rate.sleep()
        else:
            rospy.sleep(DATA_GATHERING_TIME)
        # Off subscribers
        for sensor in self.sensors:
            sensor.off()

    def write_data_to_file(self):
        counter = 0
        calculate_name = lambda i: FILE_NAME_BASE + str(i) + FILE_NAME_END
        while os.path.isfile(calculate_name(counter)) is not False:
            counter += 1
        self.fileName = calculate_name(counter)

        write_file = open(self.fileName, "w")
        new_dump = dict()

        data = dict([(Json.DATA, self.sensors[0].linear),
                     (Json.TIME, self.sensors[0].time)])
        new_dump[Json.LINEAR_SPEED] = data

        data = dict([(Json.DATA, self.sensors[1].data),
                     (Json.TIME, self.sensors[1].time)])
        new_dump[Json.LEFT_ENCODER_SPEED] = data

        data = dict([(Json.DATA, self.sensors[2].data),
                     (Json.TIME, self.sensors[2].time)])
        new_dump[Json.LEFT_MOTOR_SPEED] = data

        data = dict([(Json.DATA, self.sensors[0].angular),
                     (Json.TIME, self.sensors[0].time)])
        new_dump[Json.ANGULAR_SPEED] = data

        data = dict([(Json.DATA, self.sensors[3].data),
                     (Json.TIME, self.sensors[3].time)])
        new_dump[Json.RIGHT_ENCODER_SPEED] = data

        data = dict([(Json.DATA, self.sensors[4].data),
                     (Json.TIME, self.sensors[4].time)])
        new_dump[Json.RIGHT_MOTOR_SPEED] = data

        data = dict([(Json.DATA, self.sensors[5].x),
                     (Json.TIME, self.sensors[5].time)])
        new_dump[Json.POSITION_X] = data

        data = dict([(Json.DATA, self.sensors[5].y),
                     (Json.TIME, self.sensors[5].time)])
        new_dump[Json.POSITION_Y] = data

        data = dict([(Json.DATA, self.sensors[5].z),
                     (Json.TIME, self.sensors[5].time)])
        new_dump[Json.POSITION_DIR] = data

        json.dump(new_dump, write_file, indent=2)
        print("File with name has just been created:" + \
               os.getcwd() + "/" + self.fileName + ".")
        print("Finish.")

    def read_data_and_create_plot(self, fileName=None):
        if fileName is not None:
            self.fileName = fileName
        read_file = open(self.fileName, "r")
        data = json.load(read_file)

        name = list([Json.LINEAR_SPEED, Json.LEFT_ENCODER_SPEED,
                     Json.LEFT_MOTOR_SPEED, Json.ANGULAR_SPEED,
                     Json.RIGHT_ENCODER_SPEED, Json.RIGHT_MOTOR_SPEED,
                     Json.POSITION_X, Json.POSITION_Y,
                     Json.POSITION_DIR])
        for i in range(0, len(name)):
            plt.subplot(3, 3, i + 1)
            plt.plot(data[name[i]][Json.TIME], data[name[i]][Json.DATA], 'b')
            plt.grid()
            plt.title(name[i])
        plt.show()

    def create_desired_plot(self):
        referenceTime = list([0, 0])
        for i in range(1, DESIRED_SPEED_LENGTH):
            referenceTime.append(i * TIME_BEFORE_SPEED_CHANGE)
            referenceTime.append(i * TIME_BEFORE_SPEED_CHANGE)
        referenceTime.append(TIME_BEFORE_SPEED_CHANGE * DESIRED_SPEED_LENGTH)

        plt.subplot(3, 3, 1)
        desiredSpeeds = DESIRED_LINEAR_SPEEDS
        referenceSpeed = list([0, desiredSpeeds[0]])
        for i in range(1, DESIRED_SPEED_LENGTH):
            referenceSpeed.append(desiredSpeeds[i - 1])
            referenceSpeed.append(desiredSpeeds[i])
        referenceSpeed.append(desiredSpeeds[-1])
        plt.plot(referenceTime, referenceSpeed, 'r')

        plt.subplot(3, 3, 4)
        desiredSpeeds = DESIRED_ANGULAR_SPEEDS
        referenceSpeed = list([0, desiredSpeeds[0]])
        for i in range(1, DESIRED_SPEED_LENGTH):
            referenceSpeed.append(desiredSpeeds[i - 1])
            referenceSpeed.append(desiredSpeeds[i])
        referenceSpeed.append(desiredSpeeds[-1])
        plt.plot(referenceTime, referenceSpeed, 'r')

try:
    test = Test()
    test.do("mode")
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')
