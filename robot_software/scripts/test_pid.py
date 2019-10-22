#!/usr/bin/env python
import rospy
import json
import matplotlib.pyplot as plt
import numpy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3

# constants
SPEED_TOPIC = "/encoderLeftSpeedTopic"
CMD_TOPIC = "/cmdTopic"
THIS_NODE_NAME = 'test_pid_node'
JSON_FILE_NAME = "test_pid_result.txt"

rospy.init_node(THIS_NODE_NAME)
time = list([0])
speed = list([0])


def handle_speed(msg):
    print("received: " + str(msg))
    time.append(time[-1] + 0.1)
    speed.append(msg.data)


def start_test():
    # Initialization
    print("Node have started. Sleep for short time...")
    cmd = rospy.Publisher(CMD_TOPIC, Twist, queue_size = 10)
    rospy.sleep(0.1)
    msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    cmd.publish(msg)
    TIME_FOR_ONE_SPEED = float(2)
    rate = rospy.Rate(1 / TIME_FOR_ONE_SPEED)
    rate.sleep()

    # Gather data
    sub = rospy.Subscriber(SPEED_TOPIC, Float32, handle_speed)
    desiredSpeeds = [0.2, -0.2, 0, -0.15]
    for c in desiredSpeeds:
        msg.linear.x = c
        cmd.publish(msg)
        print("command:\n" + str(msg))
        rate.sleep()
    sub.unregister()

    # Write to file
    write_file = open(JSON_FILE_NAME, "w")
    json.dump(dict([("time", time), ("speed", speed)]), write_file, indent=2)
    print("File with name " + JSON_FILE_NAME + " has just been created.")
    print("Finish.")
    
    # Create desirable plot
    referenceTime = list([0, 0])
    referenceSpeed = list([0, desiredSpeeds[0]])
    for i in range(1, len(desiredSpeeds)):
        referenceTime.append(i * TIME_FOR_ONE_SPEED)
        referenceTime.append(i * TIME_FOR_ONE_SPEED)
        referenceSpeed.append(desiredSpeeds[i - 1])
        referenceSpeed.append(desiredSpeeds[i])
    referenceTime.append( TIME_FOR_ONE_SPEED * len(desiredSpeeds) )
    referenceSpeed.append(desiredSpeeds[-1])
    plt.plot(referenceTime, referenceSpeed)

    # Crete real plot
    plt.plot(time, speed)

    # Show plots
    plt.title('PI regulator theoretical test')
    plt.xlabel('time, s')
    plt.ylabel('speed, m/s')
    plt.grid()
    plt.show()


try:
    start_test()
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')
