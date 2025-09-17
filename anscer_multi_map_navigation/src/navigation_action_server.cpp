// src/navigation_action_server.cpp
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "anscer_multi_map_navigation/NavigateToGoalMapAction.h"
#include "anscer_multi_map_navigation/switch_map.h"
#include "anscer_multi_map_navigation/wormhole_manager.h"

class NavigationServer {
public:
  NavigationServer(const std::string &db_path);
  ~NavigationServer() = default;

private:
  // ROS
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<
      anscer_multi_map_navigation::NavigateToGoalMapAction>
      as_;
  ros::Publisher initial_pose_pub_;
  ros::Subscriber amcl_sub_;

  // helpers/managers
  WormholeManager wormhole_manager_;
  MapSwitcher map_switcher_;

  // state
  std::string current_map_;

  // overrides/defaults
  bool global_override_pose_ = false;
  double global_override_x_ = 0.0;
  double global_override_y_ = 0.0;
  double global_override_yaw_ = 0.0;

  std::unordered_map<std::string, std::tuple<double, double, double>>
      per_map_defaults_;

  // trajectory logging
  std::vector<geometry_msgs::PoseStamped> trajectory_;
  std::mutex trajectory_mutex_;
  bool logging_enabled_ = true;
  std::string amcl_topic_;
  std::string trajectory_output_dir_;

  // methods
  bool move_base_to(double x, double y, double yaw);
  void execute(
      const anscer_multi_map_navigation::NavigateToGoalMapGoalConstPtr &goal);
  void setAmclPose(double x, double y, double yaw);
  void setAmclPoseForMap(const std::string &map, double default_x,
                         double default_y, double default_yaw);
  void loadDefaultsFromYaml(const std::string &file);

  // trajectory helpers
  void
  amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void saveTrajectoryCsv(const std::string &filename);
};

NavigationServer::NavigationServer(const std::string &db_path)
    : nh_(), as_(nh_, "navigate_to_goal",
                 boost::bind(&NavigationServer::execute, this, _1), false),
      wormhole_manager_(db_path) {
  // defaults
  current_map_ = "map1";

  // publishers & subscribers
  initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "initialpose", 1, true);

  // params
  nh_.param<std::string>("amcl_pose_topic", amcl_topic_,
                         std::string("amcl_pose"));
  nh_.param("enable_trajectory_logging", logging_enabled_, true);

  // default trajectory dir inside package (overridable via param)
  nh_.param<std::string>("trajectory_output_dir", trajectory_output_dir_,
                         ros::package::getPath("anscer_multi_map_navigation") +
                             "/trajectories");

  // ensure directory exists (create if needed)
  struct stat st = {0};
  if (stat(trajectory_output_dir_.c_str(), &st) == -1) {
    if (mkdir(trajectory_output_dir_.c_str(), 0755) != 0) {
      ROS_WARN("Could not create trajectory_output_dir '%s' (check "
               "permissions). Falling back to /tmp.",
               trajectory_output_dir_.c_str());
      trajectory_output_dir_ = "/tmp";
    } else {
      ROS_INFO("Created trajectory_output_dir: %s",
               trajectory_output_dir_.c_str());
    }
  }

  amcl_sub_ =
      nh_.subscribe(amcl_topic_, 50, &NavigationServer::amclCallback, this);

  // optional global override
  nh_.param("override_initial_pose", global_override_pose_, false);
  nh_.param("override_initial_x", global_override_x_, 0.0);
  nh_.param("override_initial_y", global_override_y_, 0.0);
  nh_.param("override_initial_yaw", global_override_yaw_, 0.0);

  // Load YAML defaults (path configurable)
  std::string yaml_file;
  nh_.param<std::string>("initial_pose_yaml", yaml_file,
                         ros::package::getPath("anscer_multi_map_navigation") +
                             "/config/initial_poses.yaml");
  loadDefaultsFromYaml(yaml_file);

  // startup map
  std::string startup_map;
  nh_.param<std::string>("startup_map", startup_map, "map1");

  double default_x = 0.0, default_y = 0.0, default_yaw = 0.0;
  auto it = per_map_defaults_.find(startup_map);
  if (it != per_map_defaults_.end()) {
    std::tie(default_x, default_y, default_yaw) = it->second;
    ROS_INFO("Startup map %s defaults: (%.6f, %.6f, %.6f)", startup_map.c_str(),
             default_x, default_y, default_yaw);
  } else {
    ROS_WARN("No startup defaults found for %s, using (0,0,0).",
             startup_map.c_str());
  }

  // switch to startup map and initialize AMCL
  ROS_INFO("Switching to startup map: %s", startup_map.c_str());
  map_switcher_.switchToMap(startup_map);
  current_map_ = startup_map;
  setAmclPoseForMap(startup_map, default_x, default_y, default_yaw);

  as_.start();
  ROS_INFO(
      "Navigation Action Server started successfully. Ready to receive goals.");
}

void NavigationServer::loadDefaultsFromYaml(const std::string &file) {
  try {
    // loading the yaml files saved in config folder according to mapvalues
    YAML::Node doc = YAML::LoadFile(file);
    if (doc["initial_pose_defaults"]) {
      for (auto it = doc["initial_pose_defaults"].begin();
           it != doc["initial_pose_defaults"].end(); ++it) {
        std::string map = it->first.as<std::string>();
        double x = 0.0, y = 0.0, yaw = 0.0;
        if (it->second["x"])
          x = it->second["x"].as<double>();
        if (it->second["y"])
          y = it->second["y"].as<double>();
        if (it->second["yaw"])
          yaw = it->second["yaw"].as<double>();
        per_map_defaults_[map] = std::make_tuple(x, y, yaw);
        ROS_INFO("Loaded default initial pose for %s: (%.6f, %.6f, %.6f)",
                 map.c_str(), x, y, yaw);
      }
    } else {
      ROS_WARN("No 'initial_pose_defaults' section in YAML: %s", file.c_str());
    }
  } catch (const std::exception &e) {
    ROS_ERROR("Failed to load YAML file %s: %s", file.c_str(), e.what());
  }
}

bool NavigationServer::move_base_to(double x, double y, double yaw) {
  // sending the goal and the response is bool
  std::string move_base_name;
  nh_.param<std::string>("move_base_action_name", move_base_name,
                         std::string("move_base"));
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(
      move_base_name, true);
  if (!ac.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("Failed to connect to %s server within timeout.",
              move_base_name.c_str());
    return false;
  }

  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.frame_id = "map";
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.pose.position.x = x;
  mb_goal.target_pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();
  mb_goal.target_pose.pose.orientation = tf2::toMsg(q);

  ac.sendGoal(mb_goal);
  ac.waitForResult();

  return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void NavigationServer::setAmclPose(double x, double y, double yaw) {
  // small wait to allow the new map frame and AMCL to settle after a map
  // load/switch
  ros::Duration(0.2).sleep();

  geometry_msgs::PoseWithCovarianceStamped p;
  p.header.stamp = ros::Time::now();
  p.header.frame_id = "map";

  p.pose.pose.position.x = x;
  p.pose.pose.position.y = y;
  p.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();
  p.pose.pose.orientation = tf2::toMsg(q);

  for (int i = 0; i < 36; ++i)
    p.pose.covariance[i] = 0.0;
  p.pose.covariance[0] = 0.25;
  p.pose.covariance[7] = 0.25;
  p.pose.covariance[35] = 0.06853892326654787;

  for (int i = 0; i < 5 && ros::ok(); ++i) {
    p.header.stamp = ros::Time::now();
    initial_pose_pub_.publish(p);
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("Published AMCL initialpose: x=%.6f y=%.6f yaw=%.6f", x, y, yaw);
}

void NavigationServer::setAmclPoseForMap(const std::string &map,
                                         double default_x, double default_y,
                                         double default_yaw) {
  // setting amcl pose based on map value and current position
  std::string base = "initial_pose_overrides/" + map;
  double px = 0.0, py = 0.0, pyaw = 0.0;
  bool has_px = nh_.getParam(base + "/x", px);
  bool has_py = nh_.getParam(base + "/y", py);
  bool has_pyaw = nh_.getParam(base + "/yaw", pyaw);

  if (has_px && has_py && has_pyaw) {
    ROS_INFO("Using per-map override initial pose for map '%s': (%f,%f,%f)",
             map.c_str(), px, py, pyaw);
    setAmclPose(px, py, pyaw);
    return;
  }

  if (global_override_pose_) {
    ROS_INFO("Using global override initial pose: (%f,%f,%f)",
             global_override_x_, global_override_y_, global_override_yaw_);
    setAmclPose(global_override_x_, global_override_y_, global_override_yaw_);
    return;
  }

  setAmclPose(default_x, default_y, default_yaw);
}

void NavigationServer::amclCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  // creating a trajectory logger
  if (!logging_enabled_)
    return;
  geometry_msgs::PoseStamped p;
  p.header = msg->header;
  p.pose = msg->pose.pose;

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  trajectory_.push_back(p);
}

void NavigationServer::saveTrajectoryCsv(const std::string &filename) {
  // the trajectory cannot be interferred by anything during execution
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open trajectory file for writing: %s",
              filename.c_str());
    return;
  }

  ofs << "stamp,x,y,qx,qy,qz,qw\n";
  ofs << std::fixed << std::setprecision(6);
  for (const auto &p : trajectory_) {
    ofs << p.header.stamp.toSec() << ",";
    ofs << p.pose.position.x << "," << p.pose.position.y << ",";
    ofs << p.pose.orientation.x << "," << p.pose.orientation.y << ",";
    ofs << p.pose.orientation.z << "," << p.pose.orientation.w << "\n";
  }
  ofs.close();
  ROS_INFO("Saved trajectory (%zu poses) to %s", trajectory_.size(),
           filename.c_str());
  trajectory_.clear();
}

void NavigationServer::execute(
    const anscer_multi_map_navigation::NavigateToGoalMapGoalConstPtr &goal) {
  ROS_INFO("Received goal: target_map: %s, target_x: %f, target_y: %f, yaw: %f",
           goal->target_map.c_str(), goal->target_x, goal->target_y, goal->yaw);

  // prefer goal yaw for final leg
  double goal_yaw = goal->yaw;
  anscer_multi_map_navigation::NavigateToGoalMapResult result;

  auto save_and_tag = [&](const std::string &status) {
    if (logging_enabled_) {
      std::string fname = trajectory_output_dir_ + "/trajectory_" +
                          current_map_ + "_to_" + goal->target_map + "_" +
                          std::to_string(ros::Time::now().sec) + "_" + status +
                          ".csv";
      saveTrajectoryCsv(fname);
    }
  };

  // If already in target map: move directly using goal yaw
  if (goal->target_map == current_map_) {
    ROS_INFO("Already in target map. Moving directly to goal...");
    if (!move_base_to(goal->target_x, goal->target_y, goal_yaw)) {
      result.success = false;
      result.message = "Failed to reach goal";
      save_and_tag("aborted");
      as_.setAborted(result);
      return;
    }
    result.success = true;
    result.message = "Reached";
    save_and_tag("succeeded");
    ROS_INFO("Robot successfully reached the goal in map: %s",
             current_map_.c_str());
    as_.setSucceeded(result);
    return;
  }

  // Try direct wormhole (x,y,yaw)
  double wx, wy, wyaw;
  std::tie(wx, wy, wyaw) =
      wormhole_manager_.getWormholeToMap(current_map_, goal->target_map);
  bool direct_found = !(std::isnan(wx) || std::isnan(wy));
  if (direct_found) {
    double worm_yaw_to_use = std::isnan(wyaw) ? goal_yaw : wyaw;
    ROS_INFO("Found direct wormhole from %s to %s at (%.6f, %.6f, yaw=%.6f)",
             current_map_.c_str(), goal->target_map.c_str(), wx, wy, wyaw);

    if (!move_base_to(wx, wy, worm_yaw_to_use)) {
      result.success = false;
      result.message = "Failed to reach wormhole";
      save_and_tag("aborted");
      as_.setAborted(result);
      return;
    }

    map_switcher_.switchToMap(goal->target_map);
    current_map_ = goal->target_map;

    setAmclPoseForMap(goal->target_map, wx, wy, worm_yaw_to_use);

    if (!move_base_to(goal->target_x, goal->target_y, goal_yaw)) {
      result.success = false;
      result.message = "Failed to reach goal after switch";
      save_and_tag("aborted");
      as_.setAborted(result);
      return;
    }

    result.success = true;
    result.message = "Reached";
    save_and_tag("succeeded");
    ROS_INFO("Robot successfully reached the goal in map: %s",
             current_map_.c_str());
    as_.setSucceeded(result);
    return;
  }

  // No direct wormhole: go via map1 intermediate (unless already map1)
  if (current_map_ != "map1") {
    double ix, iy, iyaw;
    std::tie(ix, iy, iyaw) =
        wormhole_manager_.getWormholeToMap(current_map_, "map1");
    if (std::isnan(ix) || std::isnan(iy)) {
      ROS_ERROR("No wormhole from %s to map1", current_map_.c_str());
      result.success = false;
      result.message = "No wormhole to map1";
      save_and_tag("aborted");
      as_.setAborted(result);
      return;
    }

    double interm_yaw = std::isnan(iyaw) ? goal_yaw : iyaw;
    ROS_INFO("Moving from %s to map1 via (%.6f, %.6f, yaw=%.6f)",
             current_map_.c_str(), ix, iy, interm_yaw);

    if (!move_base_to(ix, iy, interm_yaw)) {
      result.success = false;
      result.message = "Failed to reach intermediate wormhole";
      save_and_tag("aborted");
      as_.setAborted(result);
      return;
    }

    map_switcher_.switchToMap("map1");
    current_map_ = "map1";

    double dflt_x = ix, dflt_y = iy, dflt_yaw = interm_yaw;
    auto it = per_map_defaults_.find("map1");
    if (it != per_map_defaults_.end())
      std::tie(dflt_x, dflt_y, dflt_yaw) = it->second;
    setAmclPoseForMap("map1", dflt_x, dflt_y, dflt_yaw);
  }

  // From map1 to final target
  double fx, fy, fyaw;
  std::tie(fx, fy, fyaw) =
      wormhole_manager_.getWormholeToMap("map1", goal->target_map);
  if (std::isnan(fx) || std::isnan(fy)) {
    ROS_ERROR("No wormhole from map1 to %s", goal->target_map.c_str());
    result.success = false;
    result.message = "No wormhole from map1 to target";
    save_and_tag("aborted");
    as_.setAborted(result);
    return;
  }

  double use_final_worm_yaw = std::isnan(fyaw) ? goal_yaw : fyaw;
  ROS_INFO("Moving from map1 to %s via (%.6f, %.6f, yaw=%.6f)",
           goal->target_map.c_str(), fx, fy, use_final_worm_yaw);

  if (!move_base_to(fx, fy, use_final_worm_yaw)) {
    result.success = false;
    result.message = "Failed to reach final wormhole";
    save_and_tag("aborted");
    as_.setAborted(result);
    return;
  }

  map_switcher_.switchToMap(goal->target_map);
  current_map_ = goal->target_map;

  double final_def_x = fx, final_def_y = fy, final_def_yaw = use_final_worm_yaw;
  auto itf = per_map_defaults_.find(goal->target_map);
  if (itf != per_map_defaults_.end())
    std::tie(final_def_x, final_def_y, final_def_yaw) = itf->second;

  setAmclPoseForMap(goal->target_map, final_def_x, final_def_y, final_def_yaw);

  if (!move_base_to(goal->target_x, goal->target_y, goal_yaw)) {
    result.success = false;
    result.message = "Failed to reach goal after final switch";
    save_and_tag("aborted");
    as_.setAborted(result);
    return;
  }

  result.success = true;
  result.message = "Reached";
  save_and_tag("succeeded");
  ROS_INFO("Robot successfully reached the goal in map: %s",
           current_map_.c_str());
  as_.setSucceeded(result);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "navigation_server_node");
  ros::NodeHandle nh;

  std::string db_path;
  nh.param<std::string>("wormhole_db_path", db_path,
                        ros::package::getPath("anscer_multi_map_navigation") +
                            "/database/wormholes.db");

  ROS_INFO("Starting navigation server with wormhole database at: %s",
           db_path.c_str());
  NavigationServer navigation_server(db_path);

  ros::spin();
  return 0;
}
