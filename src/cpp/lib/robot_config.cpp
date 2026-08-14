#include "robot_config.hpp"

#include "vendor/nlohmann/json.hpp"
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>

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
  config.protocol = j.value("protocol", "moteus");
  config.controller = j.value("controller", "angle_pid");
  config.comm_ch = j.value("comm_ch", std::vector<std::string>{"can0"});

  if (config.comm_ch.empty()) {
    throw std::runtime_error("robot_config: comm_ch must not be empty");
  }
  {
    std::set<std::string> uniq(config.comm_ch.begin(), config.comm_ch.end());
    if (uniq.size() != config.comm_ch.size()) {
      throw std::runtime_error("robot_config: duplicate netdev name in comm_ch");
    }
  }

  for (const auto& ax : j.at("axes")) {
    AxisConfig a;
    a.index = ax.at("index").get<int>();
    a.name = ax.at("name").get<std::string>();
    a.device_id = ax.at("device_id").get<int>();
    a.comm_ch = ax.value("comm_ch", 0);
    a.motdir = ax.at("motdir").get<int>();
    a.initial_position = ax.at("initial_position").get<double>();
    a.reset_position = ax.at("reset_position").get<double>();
    a.velocity_limit = ParseDouble(ax.at("velocity_limit"));
    a.accel_limit = ParseDouble(ax.at("accel_limit"));
    a.torque_limit = ax.at("torque_limit").get<double>();

    if (a.comm_ch < 0 || a.comm_ch >= static_cast<int>(config.comm_ch.size())) {
      throw std::runtime_error("robot_config: axis '" + a.name +
                               "' comm_ch index out of range: " +
                               std::to_string(a.comm_ch));
    }
    config.axes.push_back(a);
  }

  // settings 系 API (ReadParam 等) は device_id のみでモータを特定するため、
  // チャンネルをまたいだ重複も許さない (全体で一意)
  {
    std::set<int> ids;
    for (const auto& a : config.axes) {
      if (!ids.insert(a.device_id).second) {
        throw std::runtime_error("robot_config: duplicate device_id: " +
                                 std::to_string(a.device_id));
      }
    }
  }

  return config;
}

RobotConfig LoadFromFile(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    // 開けないまま空文字列を Parse すると意味不明な json parse_error になるので
    // パス入りで明示的に落とす。相対パスは daemon の作業ディレクトリ基準
    // (_unstable_deploy 入り dataflow では _work/<session-id>) なことに注意。
    throw std::runtime_error("robot_config: cannot open file: " + path);
  }
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());
  return Parse(content);
}

} // namespace robot_config