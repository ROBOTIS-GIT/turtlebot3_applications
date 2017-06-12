cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_auto_docking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  laser_filters
  laser_geometry
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtlebot3_auto_docking
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs laser_filters laser_geometry
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(get_goal_position src/get_goal_position.cpp)
 add_dependencies(get_goal_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(get_goal_position   ${catkin_LIBRARIES} )

 add_executable(goal_position_broadcaster src/goal_position_broadcaster.cpp)
 add_dependencies(goal_position_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(goal_position_broadcaster   ${catkin_LIBRARIES} )

 add_executable(turtlebot3_auto_docking src/turtlebot3_auto_docking.cpp)
 add_dependencies(turtlebot3_auto_docking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(turtlebot3_auto_docking   ${catkin_LIBRARIES} )
