cmake_minimum_required(VERSION 3.0.2)
project(anscer_multi_map_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  message_generation
  rospack
  tf2
  tf2_geometry_msgs
  roslib
)

## Find non-catkin packages
find_package(SQLite3 REQUIRED)
find_package(yaml-cpp REQUIRED)   

## Generate messages / actions
add_action_files(
  DIRECTORY action
  FILES NavigateToGoalMap.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

## Catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib actionlib_msgs move_base_msgs message_runtime roslib
  DEPENDS yaml-cpp SQLite3
)

## Includes
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}    
)

## Executables / libraries
add_executable(multi_map_navigation_action_server
  src/navigation_action_server.cpp
  src/wormhole_manager.cpp
  src/switch_map.cpp
)

add_dependencies(multi_map_navigation_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link libraries
target_link_libraries(multi_map_navigation_action_server
  ${catkin_LIBRARIES}
  sqlite3
)

# Link yaml-cpp properly depending on how the package exported it
# Modern yaml-cpp provides an imported target `yaml-cpp` (or `yaml-cpp::yaml-cpp`).
if (TARGET yaml-cpp)
  target_link_libraries(multi_map_navigation_action_server yaml-cpp)
elseif (TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(multi_map_navigation_action_server yaml-cpp::yaml-cpp)
else()
  # Fallback to variable names exported by older find modules
  if (DEFINED YAML_CPP_LIBRARIES)
    target_link_libraries(multi_map_navigation_action_server ${YAML_CPP_LIBRARIES})
  endif()
endif()

## Install
install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
