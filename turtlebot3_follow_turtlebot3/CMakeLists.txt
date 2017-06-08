cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_follow_turtlebot3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES turtlebot3_follow_turtlebot3
  CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs nav_msgs
 # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot3_follow_turtlebot3 src/turtlebot3_follow_turtlebot3.cpp)
add_dependencies(turtlebot3_follow_turtlebot3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_follow_turtlebot3 ${catkin_LIBRARIES} )

add_executable(turtlebot3_tf_broadcaster src/turtlebot3_tf_broadcaster.cpp)
add_dependencies(turtlebot3_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_tf_broadcaster ${catkin_LIBRARIES} )
