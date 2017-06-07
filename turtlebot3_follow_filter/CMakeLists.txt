cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_follow_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  laser_filters
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES follow3_demo_filter
  CATKIN_DEPENDS roscpp std_msgs laser_filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
