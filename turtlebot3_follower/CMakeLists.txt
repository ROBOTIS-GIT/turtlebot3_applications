cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_follower)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES people_detect
  CATKIN_DEPENDS 
)

catkin_install_python(PROGRAMS src/follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)



