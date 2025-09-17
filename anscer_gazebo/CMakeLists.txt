cmake_minimum_required(VERSION 2.8.3)
project(anscer_gazebo)
cmake_policy(SET CMP0054 NEW)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  anscer_description
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS GAZEBO
)
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
