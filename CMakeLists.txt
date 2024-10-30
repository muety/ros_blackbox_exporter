cmake_minimum_required(VERSION 3.0.2)
project(ros_blackbox_exporter)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

# Declare the package dependencies
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)
