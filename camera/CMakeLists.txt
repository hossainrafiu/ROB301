################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(camera)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS 
  nodes/camera_rgb.py
  nodes/camera_mono.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
