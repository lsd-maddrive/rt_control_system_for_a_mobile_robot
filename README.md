# Real-time system for mobile robot control

### How to start:

1. Clone this repository with submodules:
`git clone --recursive https://github.com/PonomarevDA/rts_for_mobile_robot_control.git`
2. Install all dependencies
`startup.sh`

### Requirements:

- [ROS Melodic](http://wiki.ros.org/melodic)

### Consists of:

* robot_firmware - Eclipse project based on ChibiOS for STM32 Nucleo144-F767ZI board
* robot_software - ROS package for raspberry PI 2B and PC
* robot_description - ROS package with robot description
* ydlidar_ros - ROS package for YDLIDAR

### Useful links

* [F767ZI Reference manual (RM)](http://www.st.com/content/ccc/resource/technical/document/reference_manual/group0/96/8b/0d/ec/16/22/43/71/DM00224583/files/DM00224583.pdf/jcr:content/translations/en.DM00224583.pdf)
* [F767ZI Datasheet (DS)](http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/c5/37/9c/1d/a6/09/4e/1a/DM00273119/files/DM00273119.pdf/jcr:content/translations/en.DM00273119.pdf)
* [F767ZI Nucleo pinout](https://os.mbed.com/platforms/ST-Nucleo-F767ZI/)
* [ydlidar x4 datasheet and manuals](http://www.ydlidar.com/download/)
* [ydlidar ros package - new repository](https://github.com/YDLIDAR/ydlidar_ros/)
* [ydlidar ros package - old repository](https://github.com/EAIBOT/ydlidar/)

### Notes

The firmware is based on the ChibiOS/RT demos for STM32 Nucleo144-F767ZI
Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.

