FROM ros:melodic
LABEL description="Tracked robot"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/tracked_robot

# 1. Install basic requirements
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git                     \
                        python-catkin-tools     \
                        python-pip              \
                        python3-pip

# 2. Install package requirements
RUN git clone https://github.com/PonomarevDA/rt_control_system_for_a_mobile_robot --recursive .
RUN sudo apt-get -y install ros-melodic-gmapping 			    \
                            ros-melodic-rosserial-python 	    \
                            ros-melodic-turtlebot3 			    \
                            ros-melodic-turtlebot3-gazebo 	    \
                            ros-melodic-dwa-local-planner   	\
                            ros-melodic-move-base 			    \
                            ros-melodic-tf					    \
                            ros-melodic-rviz                    \
                            ros-melodic-turtlesim               \
                            ros-melodic-xacro                   \
                            libsuitesparse-dev
RUN pip install -r requirements.txt
RUN git -C slam_karto/sparse_bundle_adjustment pull || git clone https://github.com/ros-perception/sparse_bundle_adjustment slam_karto/sparse_bundle_adjustment
RUN git -C slam_karto/slam_karto pull               || git clone https://github.com/ros-perception/slam_karto slam_karto/slam_karto
RUN git -C slam_karto/open_karto pull               || git clone https://github.com/ros-perception/open_karto slam_karto/open_karto
RUN git -C timed_roslaunch pull                     || git clone https://github.com/MoriKen254/timed_roslaunch.git -b melodic-devel

                            
# Build
RUN source /opt/ros/melodic/setup.bash      &&  \
    cd ../../                               &&  \
    catkin build

CMD source /opt/ros/melodic/setup.bash      &&  \
    source /catkin_ws/devel/setup.bash      &&  \
    echo "main process has been started"    &&  \
    echo "container has been finished"
