################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(lab4)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/localizer.py
  nodes/scan_angle_publisher.py
  nodes/noisy_vel_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
