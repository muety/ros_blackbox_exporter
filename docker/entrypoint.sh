#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/ros_ws/install/setup.bash

if [ -d "/home/ros_ws/custom" ]; then
    echo "Compiling custom message definitions"
    cp -r /home/ros_ws/custom/* /home/ros_ws/src/
    colcon build
fi
