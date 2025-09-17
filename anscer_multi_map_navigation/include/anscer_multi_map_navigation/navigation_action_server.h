// include/multi_map_nav/navigation_server.h
#pragma once

#include "switch_map.h"
#include "wormhole_manager.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <anscer_multi_map_navigation/NavigateToGoalMapAction.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class NavigationServer {
public:
  NavigationServer(const std::string &db_path);
  void execute(
      const anscer_multi_map_navigation::NavigateToGoalMapGoalConstPtr &goal);

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<
      anscer_multi_map_navigation::NavigateToGoalMapAction>
      as_;
  WormholeManager wormhole_manager_;
  MapSwitcher map_switcher_;
  std::string current_map_;

  bool move_base_to(double x, double y, double yaw);
};
