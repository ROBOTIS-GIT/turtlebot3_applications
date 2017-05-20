cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_follow_turtlebot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES turtlebot02_listener
  CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs nav_msgs
 # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot_follow_turtlebot src/turtlebot_follow_turtlebot.cpp)
add_dependencies(turtlebot_follow_turtlebot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot_follow_turtlebot ${catkin_LIBRARIES} )

add_executable(turtlebot_tf_broadcaster src/turtlebot_tf_broadcaster.cpp)
add_dependencies(turtlebot_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot_tf_broadcaster ${catkin_LIBRARIES} )
