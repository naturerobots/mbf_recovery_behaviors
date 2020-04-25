cmake_minimum_required(VERSION 2.8.3)
project(moveback_recovery)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  mbf_costmap_core
  mbf_msgs
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS base_local_planner costmap_2d mbf_costmap_core mbf_msgs pluginlib roscpp tf2_geometry_msgs tf2_ros
)

add_library(${PROJECT_NAME} src/moveback_recovery.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES moveback_recovery_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
