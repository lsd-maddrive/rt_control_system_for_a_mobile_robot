#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import DeleteModel

def delete_model(model_name):
    proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    proxy(model_name)

if __name__=="__main__":
    try:
        rospy.init_node('delete_model_node')
        model_name = rospy.get_param('~model_name')
        delete_model(model_name)#"turtlebot3_burger")
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logerr('Exception catched')
