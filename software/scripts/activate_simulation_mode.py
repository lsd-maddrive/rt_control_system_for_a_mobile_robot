#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8

SIMULATION_CODE = 15

rospy.init_node('starting_simulation_node')

def start_simulation():
    pubMode = rospy.Publisher('modeSelectionTopic', UInt8, queue_size = 1)
    rate = rospy.Rate(0.1)

    msgMode = UInt8()
    msgMode.data = SIMULATION_CODE
    rate.sleep()
    pubMode.publish(msgMode)

try:
    start_simulation()
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')
