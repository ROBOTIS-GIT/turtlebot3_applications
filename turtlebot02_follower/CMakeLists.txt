cmake_minimum_required(VERSION 2.8.3)
project(turtlebot02_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtlebot02_listener
  CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot02_follower src/turtlebot02_follower.cpp)
add_dependencies(turtlebot02_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot02_follower ${catkin_LIBRARIES} )
