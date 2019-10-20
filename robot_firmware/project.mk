
PROJECT_CSRC 	= 
PROJECT_CPPSRC 	= sources/control_system/control.cpp \
				  sources/control_system/pid.cpp \
				  sources/devices/leds.cpp \
				  sources/devices/adc.cpp \
				  sources/devices/motors.cpp \
				  sources/devices/encoder.cpp \
				  sources/drivers/usb.cpp \
				  sources/text.cpp \
				  sources/odometry.cpp \
				  sources/debug.cpp \
				  ros_lib/duration.cpp \
 				  ros_lib/time.cpp \
 				  sources/ros.cpp \
				  sources/main.cpp
				  


PROJECT_INCDIR	= sources \
 				  sources/devices \
 				  ros_lib \
 				  ros_lib/ros \
 				  sources/drivers \
   				  sources/control_system

PROJECT_LIBS	=

