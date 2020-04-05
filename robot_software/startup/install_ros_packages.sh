#!/bin/bash

sudo apt install ros-$ROS_DISTRO-gmapping \
				 ros-$ROS_DISTRO-rosserial-python \
				 ros-$ROS_DISTRO-turtlebot3 \
				 ros-$ROS_DISTRO-turtlebot3-gazebo \
				 ros-$ROS_DISTRO-dwa-local-planner \
				 ros-$ROS_DISTRO-move-base

git -C slam_karto/sparse_bundle_adjustment pull || git clone https://github.com/ros-perception/sparse_bundle_adjustment slam_karto/sparse_bundle_adjustment
git -C slam_karto/slam_karto pull               || git clone https://github.com/ros-perception/slam_karto slam_karto/slam_karto
git -C slam_karto/open_karto pull               || git clone https://github.com/ros-perception/open_karto slam_karto/open_karto

