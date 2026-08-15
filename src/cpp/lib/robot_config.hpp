#pragma once

#include <array>
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
  std::array<double, 3> imu_mount_rpy_deg;  // IMU 取り付け回転 RPY [deg]
                            //   v_robot = Rz(yaw)·Ry(pitch)·Rx(roll) · v_imu
                            //   (default: {0,0,0} = ロボット座標系と一致)
  std::vector<std::string> comm_ch; // SocketCAN netdev 名リスト (default: {"can0"})
                            //   例: ["can0", "can1"]。各軸は AxisConfig::comm_ch で
                            //   このリストのインデックスを指す (PEAK PCAN-M.2 等の
                            //   複数バス構成用。対応表は /proc/pcan で確認)
  std::vector<AxisConfig> axes;
};

namespace robot_config {
  RobotConfig Parse(const std::string& json_str);
  RobotConfig LoadFromFile(const std::string& path);

  // ROBOT_CONFIG 環境変数 (未設定なら default_rel) をパスに解決する。
  // 相対パスが cwd に無い場合は、実行ファイル位置から導出したリポジトリルート
  // (<root>/src/cpp/node/<name>/build/<bin> 前提) を基準に解決するので、
  // dataflow yaml にマシン依存の絶対パスを書く必要はない
  // (_unstable_deploy 入り dataflow の _work/<session> 実行対策)。
  std::string ResolveConfigPath(
      const char* default_rel = "robot_config/mimic_v2.json");
}

