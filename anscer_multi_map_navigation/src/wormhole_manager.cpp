#include "anscer_multi_map_navigation/wormhole_manager.h"

#include <limits>
#include <ros/ros.h>
#include <sqlite3.h>
#include <string>

WormholeManager::WormholeManager(const std::string &db_path) : db_(nullptr) {
  if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
    ROS_ERROR("Failed to open wormhole DB '%s': %s", db_path.c_str(),
              sqlite3_errmsg(db_));
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  } else {
    ROS_INFO("Opened wormhole DB: %s", db_path.c_str());
  }
}

WormholeManager::~WormholeManager() {
  if (db_) {
    sqlite3_close(db_);
    db_ = nullptr;
  }
}

std::tuple<double, double, double>
WormholeManager::getWormholeToMap(const std::string &current_map,
                                  const std::string &target_map) {
  const double NO_VAL = std::numeric_limits<double>::quiet_NaN();

  if (!db_) {
    ROS_ERROR("Wormhole DB is not open. Cannot query wormholes.");
    return {NO_VAL, NO_VAL, NO_VAL};
  }

  double x = NO_VAL;
  double y = NO_VAL;
  double yaw = NO_VAL;

  const char *sql = "SELECT x, y, yaw FROM wormholes WHERE from_map = ? AND "
                    "to_map = ? LIMIT 1;";
  sqlite3_stmt *stmt = nullptr;

  ROS_DEBUG("Preparing wormhole query: %s", sql);
  int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    ROS_ERROR("Failed to prepare wormhole query: %s", sqlite3_errmsg(db_));
    return {NO_VAL, NO_VAL, NO_VAL};
  }

  sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_TRANSIENT);

  int step_result = sqlite3_step(stmt);
  if (step_result == SQLITE_ROW) {
    x = sqlite3_column_double(stmt, 0);
    y = sqlite3_column_double(stmt, 1);
    if (sqlite3_column_type(stmt, 2) != SQLITE_NULL) {
      yaw = sqlite3_column_double(stmt, 2);
    } else {
      yaw = NO_VAL;
    }
    ROS_INFO("Wormhole found from '%s' to '%s' -> (x=%.6f, y=%.6f, yaw=%s)",
             current_map.c_str(), target_map.c_str(), x, y,
             std::isnan(yaw) ? "NaN" : std::to_string(yaw).c_str());
  } else if (step_result == SQLITE_DONE) {
    ROS_DEBUG("No wormhole row for %s -> %s", current_map.c_str(),
              target_map.c_str());
  } else {
    ROS_ERROR("Error executing wormhole query: %s", sqlite3_errmsg(db_));
  }

  sqlite3_finalize(stmt);
  return {x, y, yaw};
}
