#pragma once

#include <string>
#include <vector>

struct AxisConfig {
  int index;
  std::string name;
  int device_id;
  int comm_ch;              // RobotConfig::comm_ch のインデックス (default: 0)
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
  std::string transport;    // "socketcan" or "dummy"
  std::string protocol;     // "moteus" or "foctive" (default: "moteus")
  std::string controller;   // "angle_pid" etc. (default: "angle_pid")
  std::vector<std::string> comm_ch; // SocketCAN netdev 名リスト (default: {"can0"})
                            //   例: ["can0", "can1"]。各軸は AxisConfig::comm_ch で
                            //   このリストのインデックスを指す (PEAK PCAN-M.2 等の
                            //   複数バス構成用。対応表は /proc/pcan で確認)
  std::vector<AxisConfig> axes;
};

namespace robot_config {
  RobotConfig Parse(const std::string& json_str);
  RobotConfig LoadFromFile(const std::string& path);
}

