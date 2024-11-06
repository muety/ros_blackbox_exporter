#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/ros_ws/devel/setup.bash

if [ -d "/home/ros_ws/custom" ]; then
    echo "Compiling custom message definitions"
    cp -r /home/ros_ws/custom/* /home/ros_ws/src/
    catkin_make
fi
