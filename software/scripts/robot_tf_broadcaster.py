#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

nodeName = "robot_tf_broadcaster"
publisherName = "odom"
subscriberName = "input_pose"
frameId = "base_link"
childFrameId = "map"

rospy.init_node(nodeName)
odom_pub = None
br = tf.TransformBroadcaster()

def handle_pose(msg):
    odom_quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.z)
    current_time = rospy.Time.now()
    br.sendTransform((msg.x, msg.y, 0),
                     odom_quaternion,
                     current_time,
                     frameId,
                     childFrameId)

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = frameId
    odom.pose.pose = Pose(Point(msg.x, msg.y, 0.), Quaternion(*odom_quaternion))
    odom.child_frame_id = childFrameId
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    odom_pub.publish(odom)

rospy.Subscriber(subscriberName, Point32, handle_pose)
odom_pub = rospy.Publisher(publisherName, Odometry, queue_size=50)
rospy.spin()
