# Real-time system for mobile robot control

## Brief

This project implements a real time control system for a tracked robot.

It consist of two parts:
- The firmware implemented a low level control system is based on the [ChibiOS/RT](http://chibios.org/dokuwiki/doku.php) demos for STM32 Nucleo144-F767ZI.
- The software implemented a high level control system is based on the [ROS Melodic](http://wiki.ros.org/melodic) for Raspberry PI and desktop. It mainly uses `move_base` and different SLAM packages: `slam_karto`, `gmapping` and `hector_mapping`.

## Hardware Requirements

The main hardware components are:
- Raspberry PI 2B (RPI),
- STM32 Nucleo144-F767ZI,
- motors with encoders GM25-370,
- lidar ydlidar x4.

## Software Requirements

- It is assumed you use ubuntu 18.04 with ROS melodic insalled in the way described in [the official tutorial](http://wiki.ros.org/melodic/Installation/Ubuntu) on your computer.

- The RPI is based on Ubuntu Mate 18.04.

- Python 2.7

### Installation and Building:

You can build docker image using [build script](scripts/docker/build_image.sh) or install and build manually using instruction from the [Dockerfile](Dockerfile).

### Real usage

1. You should create symlinks for stm32 and ydlidar:

```
sudo ./init_stm32_and_ydlidar.sh
```

2. Then setup `ROS_MASTER` and `ROS_IP` on RPI and PC:

`source init_env_vars.launch`

3. Run SLAM or Localization on RPI:

```
roslaunch wr8_software real_slam.launch
```

or

```
roslaunch wr8_software real_localization.launch
```

4. Run `RVUZ` on desktop to monitor the robot position and to setup desired goals:

```
roslaunch wr8_software rviz.launch
```

### Usage in Gazebo simulator

1. Run on the first terminal session:

```
roslaunch wr8_software gz_server.launch
```

2. Run on the second terminal session to spawn a robot with SLAM or Localization algorithm:

```
roslaunch wr8_software gz_slam.launch
```

or

```
roslaunch wr8_software gz_localization.launch
```

Here launching is divided into two files to have an ability to spawn several robots simultaniously. But this feature is not ready yet.

### Troubleshooting

If you have a problem with urdf model visualization in rviz in ros melodic, read [this comment](https://github.com/ros-visualization/rviz/issues/1249#issuecomment-403351217).

## Firmware
1. Install some linux distributive, for example ubuntu melodic
2. Install st-link, arm compiler, eclipse and chibios like [this tutorial](https://github.com/KaiL4eK/STM32_ChibiOS/blob/stable_17.6.x/Startup/Linux_setup.md)

### Repository content

* robot_firmware - Eclipse project based on ChibiOS for STM32 Nucleo144-F767ZI board
* robot_software - ROS package for raspberry PI 2B and PC that containts navigation stack, robot configuration and simulation scripts 
* robot_description - ROS package with robot model description
* ydlidar_ros - ROS package that has YDLIDAR driver

### Useful links
* [IEEE Real-time Control System for a Tracked Robot article](https://ieeexplore.ieee.org/document/9039168)
* [F767ZI Reference manual (RM)](http://www.st.com/content/ccc/resource/technical/document/reference_manual/group0/96/8b/0d/ec/16/22/43/71/DM00224583/files/DM00224583.pdf/jcr:content/translations/en.DM00224583.pdf)
* [F767ZI Datasheet (DS)](http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/c5/37/9c/1d/a6/09/4e/1a/DM00273119/files/DM00273119.pdf/jcr:content/translations/en.DM00273119.pdf)
* [F767ZI Nucleo pinout](https://os.mbed.com/platforms/ST-Nucleo-F767ZI/)
* [ydlidar x4 datasheet and manuals](http://www.ydlidar.com/download/)
* [ydlidar ros package - new repository](https://github.com/YDLIDAR/ydlidar_ros/)
* [ydlidar ros package - old repository](https://github.com/EAIBOT/ydlidar/)
