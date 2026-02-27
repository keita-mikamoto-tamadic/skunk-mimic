#pragma once

#include <string>
#include <vector>

struct AxisConfig {
  int index;
  std::string name;
  int device_id;
  int motdir;
  double initial_position;
  double reset_position;
  double velocity_limit;
  double accel_limit;
  double torque_limit;
};

struct RobotConfig {
  std::string robot_name;
  int axis_count;
  double interpolation_time;
  std::string transport;  // "socketcan" or "dummy"
  std::vector<AxisConfig> axes;
};

namespace robot_config {
  RobotConfig Parse(const std::string& json_str);
  RobotConfig LoadFromFile(const std::string& path);
}

