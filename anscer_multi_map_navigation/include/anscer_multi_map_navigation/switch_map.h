#pragma once
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <string>

class MapSwitcher {
public:
  MapSwitcher();
  void switchToMap(const std::string &map_name);

private:
  ros::NodeHandle nh_;
  std::string map_folder_;
};
