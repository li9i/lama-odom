cmake_minimum_required(VERSION 2.8.3)
project(odom_logger)
add_compile_options(-std=c++11 -Ofast)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  message_generation
)
catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  message_runtime
  tf2
  tf2_msgs
  tf2_ros
  INCLUDE_DIRS include
)
INCLUDE_DIRECTORIES(include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3/)
# sm ---------------------------------------------------------------------------
add_library(odom_logger src/odom_logger.cpp)
target_link_libraries(odom_logger
  ${catkin_LIBRARIES})
add_dependencies(odom_logger
  ${catkin_EXPORTED_TARGETS})
# The executable ---------------------------------------------------------------
add_executable(odom_logger_node
  src/odom_logger_node.cpp)

target_link_libraries(odom_logger_node
  odom_logger
  ${catkin_LIBRARIES})

add_dependencies(odom_logger_node
  ${catkin_EXPORTED_TARGETS}
)
