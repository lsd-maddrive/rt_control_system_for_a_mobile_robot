#!/usr/bin/env python
import rospy
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point32
from turtlesim.msg import Pose

rospy.init_node('tf_turtle')
turtlename = rospy.get_param('~turtle_tf_name')

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     quaternion_from_euler(0, 0, msg.z),
                     rospy.Time.now(),
                     turtlename,
                     "laser_frame") # temporary!

rospy.Subscriber('input_pose',
                 Point32,
                 handle_turtle_pose)
rospy.spin()
