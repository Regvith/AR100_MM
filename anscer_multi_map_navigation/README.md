# anscer_multi_map_navigation

A ROS package for **multi-map navigation** with wormholes, designed for mobile robots (e.g. AR100).  
Each room is mapped separately, and the robot uses **overlapping regions ("wormholes")** stored in an SQLite database to switch maps seamlessly.

>  The core AMR functionality (e.g. navigation stack integration) is based on the  
> [Anscer Robotics AR100 repository](https://github.com/anscer/AR100).  
> This package extends it with multi-map navigation, wormhole management using sql schema, AMCL re-initialization, trajectory_collection and visualization.

>Terminal commands: cd ~/catkin_ws/src
git clone https://github.com/Regvith/AR100_MM.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# ALGORITHM
parse the database file
check
if target_map == current_map:
    send goal directly to move_base
else if direct wormhole exists:
    move to wormhole
    switch map
    reinitialize AMCL
    move to final goal
else:
    navigate via map1 as intermediate

# There are only two maps in this project
> map1 and map2
These maps were mapped using gmapping that was given by the AR100 repository mentioned above.

# To launch the gazebo file

> terminal command: roslaunch start_anscer start_anscer.launch

This starts the gazebo simulation and also the roscore master 

# Then launch the below file in another terminal
> terminal command: roslaunch anscer_multi_map_navigation anscer_multi_map_server.launch

This starts the navigation server which is responsible for path planning, obstacle avoidance and amcl_localization, 
and the multi_map_navigation node, which is an action server written in c++, that relies on dependencies:
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
  SQLite3
  yaml-cpp

# schema
-- wormholes_schema.sql

>> CREATE TABLE IF NOT EXISTS wormholes (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  from_map TEXT NOT NULL,
  to_map   TEXT NOT NULL,
  x REAL NOT NULL,
  y REAL NOT NULL,
  yaw REAL DEFAULT 0.0
);

>> INSERT INTO wormholes (from_map,to_map,x,y,yaw) VALUES ('map1','map2', 10.0, 1.0, 0.00);
>> INSERT INTO wormholes (from_map,to_map,x,y,yaw) VALUES ('map2','map1', 10.0, 1.5, -3.1457);

> More maps can be included by using the INSERT command, after saving maps.

# Test commands for testing the server code
Note >>>>> The robot uses absolute odometry and not relative odometry, so in this project the full map was split into two parts. 
So values must be above x >10 for map2 and below x <10 for map1 <<<<<

> For map1 -> map2  
> terminal command: rostopic pub /navigate_to_goal/goal anscer_multi_map_navigation/NavigateToGoalMapActionGoal "{ header: {stamp: now, frame_id: ''}, goal_id: {stamp: now, id: 'test_goal_1'}, goal: { target_x: 14.0, target_y: 3.0, yaw: 0.0, target_map: 'map2' } }"

>For map2->map2
> terminal command: rostopic pub /navigate_to_goal/goal anscer_multi_map_navigation/NavigateToGoalMapActionGoal "{ header: {stamp: now, frame_id: ''}, goal_id: {stamp: now, id: 'test_goal_2'}, goal: { target_x: 10.0, target_y: 2.0, yaw: 0.0, target_map: 'map2' } }"

> For map2 -> map1
> terminal command:  rostopic pub /navigate_to_goal/goal anscer_multi_map_navigation/NavigateToGoalMapActionGoal "{ header: {stamp: now, frame_id: ''}, goal_id: {stamp: now, id: 'test_goal_3'}, goal: { target_x: 1.0, target_y: -2.5, yaw: 0.0, target_map: 'map1' } }"

> For map1->map1
> terminal command: rostopic pub /navigate_to_goal/goal anscer_multi_map_navigation/NavigateToGoalMapActionGoal "{ header: {stamp: now, frame_id: ''}, goal_id: {stamp: now, id: 'test_goal_1'}, goal: { target_x: 1.0, target_y: 1.5, yaw: 0.0, target_map: 'map1' } }"

# For visualization of graph use
> terminal command: cd /catkin_ws/src/AR100/anscer_multi_map_navigation/scripts
> terminal command: python3 plot_latest_trajectory.py
