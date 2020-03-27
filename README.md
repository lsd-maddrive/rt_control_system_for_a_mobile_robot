# Real-time system for mobile robot control

### Description

This project consist of firmware and software for real time system for tracked mobile robot control.
- The firmware is based on the [ChibiOS/RT](http://chibios.org/dokuwiki/doku.php) demos for STM32 Nucleo144-F767ZI
- The software is based on [ROS Melodic](http://wiki.ros.org/melodic) for Raspberry PI and desktop

### Preparation for work:
##### 1. [Installing ubuntu melodic and environments setup](http://wiki.ros.org/melodic/Installation/Ubuntu)
In result, `echo $ROS_DISTRO` should return `melodic`
##### 2. Workspace preparation
1. Create workspace with any name (for example rts_for_robot_ws) and clone this repository to [your workspace]/src folder:
```
mkdir -p ~/rts_for_robot_ws/src
cd rts_for_robot_ws/src/
git clone --recursive https://github.com/PonomarevDA/rts_for_mobile_robot_control.git .
```
Note that you should use --recursive to fetch submodules and use dot in the end of last command so as not to create a directory rts_for_robot_ws inside src folder.

2. Build the workspace code and add automatic workspace settings to ~/.bashrc
```
cd ..
catkin_make
. ~/rts_for_robot_ws/devel/setup.bash
echo ". ~/rts_for_robot_ws/devel/setup.bash" >> ~/.bashrc
```
You can check result using `echo $ROS_PACKAGE_PATH`. This command should return 2 paths for system and user workspaces. For example:
/home/ubuntu/rts_for_robot_ws/src:/opt/ros/melodic/share

3. Install all dependencies
```
cd src/
./startup.sh
```

### How to start robot

1. Select RPI to run master, set ROS_MASTER_URI and ROS_IP on RPI and PC. You can use `init_env_vars.launch` in this way:

`. ./init_env_vars.launch`

2. Run `real_slam.launch` or `real_localization.launch` on RPI
3. Run `rviz.launch` on desktop

### How to start simulation

1. Run on first terminal session: `gz_server`
2. Run on second terminal session: `gz_slam.launch` or `gz_localization.launch`


### Troubleshooting

If you have problem with urdf model visualization in rviz in ros melodic, read [this comment](https://github.com/ros-visualization/rviz/issues/1249#issuecomment-403351217).

## Firmware
1. Install some linux distributive, for example ubuntu melodic
2. Install st-link, arm compiler, eclipse and chibios like [this tutorial](https://github.com/KaiL4eK/STM32_ChibiOS/blob/stable_17.6.x/Startup/Linux_setup.md)

### Repository content

* robot_firmware - Eclipse project based on ChibiOS for STM32 Nucleo144-F767ZI board
* robot_software - ROS package for raspberry PI 2B and PC that containts navigation stack, robot configuration and simulation scripts 
* robot_description - ROS package with robot model description
* ydlidar_ros - ROS package that has YDLIDAR driver

### Useful links

* [F767ZI Reference manual (RM)](http://www.st.com/content/ccc/resource/technical/document/reference_manual/group0/96/8b/0d/ec/16/22/43/71/DM00224583/files/DM00224583.pdf/jcr:content/translations/en.DM00224583.pdf)
* [F767ZI Datasheet (DS)](http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/c5/37/9c/1d/a6/09/4e/1a/DM00273119/files/DM00273119.pdf/jcr:content/translations/en.DM00273119.pdf)
* [F767ZI Nucleo pinout](https://os.mbed.com/platforms/ST-Nucleo-F767ZI/)
* [ydlidar x4 datasheet and manuals](http://www.ydlidar.com/download/)
* [ydlidar ros package - new repository](https://github.com/YDLIDAR/ydlidar_ros/)
* [ydlidar ros package - old repository](https://github.com/EAIBOT/ydlidar/)


