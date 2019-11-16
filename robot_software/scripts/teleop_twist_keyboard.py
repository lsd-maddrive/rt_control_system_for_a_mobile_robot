#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, UInt8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep
import numpy as np
import threading
import sys, select, termios, tty

helloMessage = """
Reading from keyboard and publish to /cmdTopic.
w/s - increase/decrease linear speed
a/d - increase angular speed (counterclockwise/clockwise rotation)
ctrl + c - stop and leave
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def fixBorders(speed):
    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    return speed


if __name__=="__main__":
    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    settings = termios.tcgetattr(sys.stdin)
    linearSpeed = 0
    angularSpeed = 0
    print(helloMessage)
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w':
            linearSpeed = fixBorders(linearSpeed + 10)
            angularSpeed = 0
            print("w: linearSpeed = %d" % linearSpeed)
        elif key == 's':
            linearSpeed = fixBorders(linearSpeed - 10)
            angularSpeed = 0
            print("s: linearSpeed = %d" %  linearSpeed)
        elif key == 'a':
            linearSpeed = 0
            angularSpeed = fixBorders(angularSpeed + 10)
            print("a: angularSpeed = %d" % angularSpeed)
        elif key == 'd':
            linearSpeed = 0
            angularSpeed = fixBorders(angularSpeed - 10)
            print("d: angularSpeed = %d" % angularSpeed)
        elif key == '\x03':
            print('Exit')
            break
        else:
            print('wrong key')
        vel_msg = Twist()
        PERCENTAGE_COEFFICIENT = 0.05
        vel_msg.linear.x = linearSpeed * PERCENTAGE_COEFFICIENT
        vel_msg.angular.z = angularSpeed * PERCENTAGE_COEFFICIENT
        pub.publish(vel_msg)

        sleep(0.3)
