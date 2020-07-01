#!/bin/bash
echo "Evironment variables have been set. Now they are:"

my_ip=$(hostname -I)
export LC_NUMERIC="en_US.UTF-8"
export ROS_MASTER_URI=http://ubuntu-desktop.local:11311
export ROS_IP=$my_ip

echo "LC_NUMERIC =" $LC_NUMERIC
echo "ROS_MASTER_URI =" $ROS_MASTER_URI
echo "ROS_IP =" $ROS_IP
