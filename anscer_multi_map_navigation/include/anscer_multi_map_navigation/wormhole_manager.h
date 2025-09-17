#pragma once

#include <ros/ros.h>
#include <sqlite3.h>
#include <string>
#include <tuple>

class WormholeManager {
public:
  explicit WormholeManager(const std::string &db_path);
  ~WormholeManager();

  // Returns (x, y, yaw). If not found any field may be NaN to indicate missing.
  std::tuple<double, double, double>
  getWormholeToMap(const std::string &current_map,
                   const std::string &target_map);

private:
  sqlite3 *db_;
};
