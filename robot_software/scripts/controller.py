#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from math import cos, sin

nodeName = "controller"
publisherName = "odom"
subscriberName = "cmdTopic"
frameId = "map"
childFrameId = "base_link"

rospy.init_node(nodeName)
odom_pub = None

previous_linear = 0
previous_angular = 0
previous_time = rospy.Time.now()

pose_dir = float()
pose_x = float()
pose_y = float()

def handle_pose(msg):
    global frameId, childFrameId, odom_pub
    global previous_linear, previous_angular, previous_time, pose_dir, pose_x, pose_y

    current_time = rospy.Time.now()
    delta_time = (current_time.nsecs - previous_time.nsecs) * 0.000000001 + (current_time.secs - previous_time.secs)
    previous_time = current_time

    delta_path = previous_linear * delta_time
    pose_dir = pose_dir + previous_angular * delta_time
    pose_x = pose_x + delta_path * cos(pose_dir)
    pose_y = pose_y + delta_path * sin(pose_dir)

    previous_linear = msg.linear.x
    previous_angular = msg.angular.z

    print("command: ", previous_linear, "/", previous_angular, ", pose now = ", pose_x, pose_y, pose_dir)

    odom_quaternion = tf.transformations.quaternion_from_euler(0, 0, pose_dir)

    odom = Odometry()
    odom.pose.pose = Pose(Point(pose_x, pose_y, 0.), Quaternion(*odom_quaternion))
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    odom.header.stamp = current_time
    odom.header.frame_id = frameId
    odom.child_frame_id = childFrameId
    odom_pub.publish(odom)
    
    br = tf.TransformBroadcaster()
    br.sendTransform((pose_x, pose_y, 0),
                     odom_quaternion,
                     current_time,
                     childFrameId,
                     frameId)

print("Controller enabled.")
previous_time = rospy.Time.now()
rospy.Subscriber(subscriberName, Twist, handle_pose)
odom_pub = rospy.Publisher(publisherName, Odometry, queue_size=50)
rospy.spin()
