#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>

const static std::string nodeName = "state_listener";
const static std::string topicName = "positionTopic";

const static std::string parentFrameId = "map";
const static std::string childFrameId = "base_link";

void topicCallback(const geometry_msgs::Point32& msg)
{
	static tf::TransformBroadcaster broadcaster;
	static geometry_msgs::TransformStamped odom_trans;

	double x = msg.x;
	double y = msg.y;
	double angle = msg.z;

	odom_trans.header.frame_id = parentFrameId;
	odom_trans.child_frame_id = childFrameId;

	ROS_INFO("I heard: [%f][%f][%f]", msg.x, msg.y, msg.z);

	odom_trans.header.stamp = ros::Time::now();
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);
	broadcaster.sendTransform(odom_trans);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, nodeName);
    ros::NodeHandle n;
	uint32_t queueSize = 1000;
	ros::Subscriber sub = n.subscribe(topicName, queueSize, topicCallback);
	ros::spin();
    return 0;
}
