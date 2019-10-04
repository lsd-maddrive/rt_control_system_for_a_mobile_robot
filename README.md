# Real-time system for mobile robot control

### Description

This project consist of firmware and software for real time system for tracked mobile robot control.
- The firmware is based on the [ChibiOS/RT](http://chibios.org/dokuwiki/doku.php) demos for STM32 Nucleo144-F767ZI
- The software is based on [ROS Melodic](http://wiki.ros.org/melodic) for Raspberry PI and desktop

### Software installation

1. Clone this repository with submodules:
`git clone --recursive https://github.com/PonomarevDA/rts_for_mobile_robot_control.git`
2. Install all dependencies
`startup.sh`

### How to start robot

1. Select RPI to run master, set ROS_MASTER_URI and ROS_IP on RPI and PC. You can use `init_env_vars.launch` in this way:

`. ./init_env_vars.launch`

2. Run `rpi_start_slam.launch` or `rpi_start_localization.launch` on RPI
3. Run `rviz.launch` on desktop

### How to start simulation

1. Run `pc_start_sim_slam.launch` or `pc_start_sim_localization.launch` on PC

### Troubleshooting

If you have problem with urdf model visualization in rviz in ros melodic, read [this comment](https://github.com/ros-visualization/rviz/issues/1249#issuecomment-403351217).

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


