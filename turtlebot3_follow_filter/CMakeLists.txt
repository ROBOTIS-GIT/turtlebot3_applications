cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_follow_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  laser_filters
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES turtlebot3_follow_filter
  CATKIN_DEPENDS roscpp std_msgs laser_filters
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
