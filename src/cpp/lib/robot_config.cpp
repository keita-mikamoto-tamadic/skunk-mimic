#include "robot_config.hpp"

#include "vendor/nlohmann/json.hpp"
#include <fstream>
#include <limits>

namespace robot_config {
namespace {
  double ParseDouble(const nlohmann::json& j) {
    if (j.is_string()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return j.get<double>();
  }
} // anonymous namespace

RobotConfig Parse(const std::string& json_str) {
  auto j = nlohmann::json::parse(json_str);
  
  RobotConfig config;
  config.robot_name = j.at("robot_name").get<std::string>();
  config.axis_count = j.at("axis_count").get<int>();
  config.interpolation_time = j.at("interpolation_time").get<double>();
  config.transport = j.value("transport", "socketcan");
  config.controller = j.value("controller", "angle_pid");

  for (const auto& ax : j.at("axes")) {
    AxisConfig a;
    a.index = ax.at("index").get<int>();
    a.name = ax.at("name").get<std::string>();
    a.device_id = ax.at("device_id").get<int>();
    a.motdir = ax.at("motdir").get<int>();
    a.initial_position = ax.at("initial_position").get<double>();
    a.reset_position = ax.at("reset_position").get<double>();
    a.velocity_limit = ParseDouble(ax.at("velocity_limit"));
    a.accel_limit = ParseDouble(ax.at("accel_limit"));
    a.torque_limit = ax.at("torque_limit").get<double>();
    config.axes.push_back(a);
  }

  return config;
}

RobotConfig LoadFromFile(const std::string& path) {
  std::ifstream ifs(path);
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());
  return Parse(content);
}

} // namespace robot_config