# AR100 Simulation Repository

Welcome to the ANSCER (Autonomous Mobile Robot) Simulation Repository! This repository contains packages and instructions to set up and run a simulation of an autonomous mobile robot in a ROS (Robot Operating System) environment.

## Table of Contents
- [Packages](#packages)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Creating a Map](#creating-a-map)
- [Navigation](#navigation)

## Packages
The ANSCER Simulation Repository consists of the following ROS packages:
- **anscer_description**: Contains the robot's URDF (Unified Robot Description Format) files for visualization.
- **anscer_gazebo**: Provides Gazebo simulation configurations for the ANSCER robot.
- **anscer_navigation**: Configures and launches the navigation stack for autonomous robot navigation.
- **anscer_slam**: Configures and launches the Simultaneous Localization and Mapping (SLAM) system for mapping the environment.
- **anscer_teleop**: Allows teleoperation of the robot for manual control.
- **start_anscer**: A convenience package to start the simulation environment.

## Installation
Before you can use this repository, you need to have ROS (Robot Operating System) installed. If you haven't installed ROS yet, you can follow the installation instructions provided on the official ROS website: [ROS Installation Instructions](http://wiki.ros.org/Installation).

To get started with the ANSCER Simulation, follow these steps:

1. Let's create and build a catkin workspace:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ..
    catkin_make
3. Clone this repository to your local machine:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/anscer/AR100.git
4. Navigate to your catkin workspace and build the packages. If you haven't set up a catkin workspace, you can follow the ROS documentation to create one.
    ```bash
    cd ~/catkin_ws
    catkin_make
    # If needed, use the following command to build individual packages
    # catkin_make -DCATKIN_WHITELIST_PACKAGES="package1;package2"

## Running the Simulation
Follow these steps to start the ANSCER simulation:

1. Launch Gazebo to visualize the robot and its environment:
    ```bash
    roslaunch start_anscer start_anscer.launch
2. To teleoperate the robot, open a new terminal and launch the teleoperation package:
    ```bash
    roslaunch anscer_teleop anscer_teleop_key.launch

## Creating a Map
You can create a map of the environment using the Simultaneous Localization and Mapping (SLAM) package. Follow these steps:

1. Launch the SLAM system:
    ```bash
    roslaunch anscer_slam anscer_slam.launch
2. Use the teleoperation package to manually explore the environment and build the map.
3. Save the map using the map_server package:
    ```bash
    rosrun map_server map_saver -f ~/<workspace_name>/src/AR100/anscer_navigation/maps/<map_name>
    # Replace workspace_name with the name of the workspace you have created.
    # Replace map_name with the name of the map you want to provide.
4. Once the map is saved, you can stop the anscer_slam package.

## Navigation
To enable autonomous navigation using the created map, follow these steps:

1. Launch the navigation stack:
    ```bash
    roslaunch anscer_navigation anscer_navigation.launch map_name:=<map_name>
    #  Replace map_name with the name of the map you want to use.
2. Use RViz to initialize the robot's pose and provide navigation goals.

    With these instructions, you should have the ANSCER simulation up and running, allowing you to explore autonomous mobile robot capabilities within the ROS environment. Happy robot simulating!
