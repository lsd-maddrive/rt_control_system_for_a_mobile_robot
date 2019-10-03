#!/bin/bash

echo "Script pc_init_bash.sh has been runned:"

echo "ROS_MASTER_URI was" $ROS_MASTER_URI "."
echo "ROS_IP was" $ROS_IP "."

export ROS_MASTER_URI=http://ubuntu-desktop.local:11311
export ROS_IP=$HOSTNAME

echo "ROS_MASTER_URI is" $ROS_MASTER_URI "now."
echo "ROS_IP is" $ROS_IP "now."
