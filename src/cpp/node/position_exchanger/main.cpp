/**
 * position_exchanger: leader 軸の位置を follower 軸へ渡す (FOCTIVE バイラテ試験用)
 *
 *   device_control_manager ──motor_status──▶ position_exchanger ──motor_commands──▶ device_control_manager
 *   foctive_web_gui ──enable (uint8: 1=開始 / 0=停止)──┘
 *
 * motor_status 駆動 (DCM の tick 周期)。ロジックは PositionExchanger (dora 非依存)。
 * 起動直後は停止 (全軸 OFF)。enable=1 を受けるまで動かない。
 *
 * 環境変数:
 *   ROBOT_CONFIG          robot_config JSON (相対パスはリポジトリルート基準)
 *   PX_LEADER_ID          leader の device_id   (既定 2, 電流 0 指令で脱力)
 *   PX_FOLLOWER_ID        follower の device_id (既定 1, POSITION_PD で追従)
 *   PX_KP_SCALE           imp_kp へのスケール (既定 1.0)
 *   PX_KD_SCALE           imp_kd へのスケール (既定 1.0)
 *   PX_ACCEL_LIMIT        follower の台形プロファイル加速度制限 [rad/s^2] (既定 1000)
 *   PX_FORCE_GAIN         力帰還ゲイン。leader へ -gain * follower トルクを返す (既定 0 = 脱力)
 *   PX_FORCE_LIMIT        leader へ返すトルクの上限 [Nm] (既定 1.0)
 */
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include "dora-node-api.h"
#include "position_exchanger.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/dora_helpers.hpp"

constexpr const char* kInputMotorStatus = "motor_status";
constexpr const char* kInputEnable = "enable";
constexpr const char* kOutputMotorCommands = "motor_commands";

static double EnvDouble(const char* name, double def) {
  const char* v = std::getenv(name);
  return v ? std::strtod(v, nullptr) : def;
}

static int EnvInt(const char* name, int def) {
  const char* v = std::getenv(name);
  return v ? std::atoi(v) : def;
}

int main() {
  auto node = init_dora_node();

  auto config = robot_config::LoadFromFile(
      robot_config::ResolveConfigPath("robot_config/foctive_bilateral_control.json"));

  PositionExchanger::Params params;
  params.leader_device_id = EnvInt("PX_LEADER_ID", params.leader_device_id);
  params.follower_device_id = EnvInt("PX_FOLLOWER_ID", params.follower_device_id);
  params.kp_scale = EnvDouble("PX_KP_SCALE", params.kp_scale);
  params.kd_scale = EnvDouble("PX_KD_SCALE", params.kd_scale);
  params.accel_limit = EnvDouble("PX_ACCEL_LIMIT", params.accel_limit);
  params.force_gain = EnvDouble("PX_FORCE_GAIN", params.force_gain);
  params.force_limit = EnvDouble("PX_FORCE_LIMIT", params.force_limit);

  PositionExchanger px;
  if (!px.Configure(config, params)) {
    std::cerr << "leader(device " << params.leader_device_id << ") / follower(device "
              << params.follower_device_id << ") が config に無い (" << config.robot_name
              << ")" << std::endl;
    return 1;
  }
  std::cout << "position_exchanger: leader=" << params.leader_device_id
            << " follower=" << params.follower_device_id
            << " kp_scale=" << params.kp_scale << " kd_scale=" << params.kd_scale
            << " accel_limit=" << params.accel_limit
            << " force_gain=" << params.force_gain
            << " force_limit=" << params.force_limit << std::endl;

  bool was_engaged = false;

  while (true) {
    auto event = node.events->next();
    auto type = event_type(event);

    if (type == DoraEventType::Stop || type == DoraEventType::AllInputsClosed) {
      std::cout << "stopping" << std::endl;
      break;
    }
    if (type != DoraEventType::Input) continue;

    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    auto info = event_as_arrow_input_with_info(
        std::move(event),
        reinterpret_cast<uint8_t*>(&c_array),
        reinterpret_cast<uint8_t*>(&c_schema));
    std::string id(info.id);

    auto import_result = arrow::ImportArray(&c_array, &c_schema);
    if (!import_result.ok()) continue;
    auto arr = std::static_pointer_cast<arrow::UInt8Array>(import_result.ValueOrDie());

    if (id == kInputEnable) {
      const bool on = ReceiveValue<uint8_t>(arr) != 0;
      px.SetEnabled(on);
      std::cout << (on ? "enable" : "disable -> all OFF") << std::endl;
      if (!on) {  // 次の motor_status を待たずに OFF を出す
        ZeroCopySendStructArray(node, kOutputMotorCommands, px.GetCommands());
      }
    }
    else if (id == kInputMotorStatus) {
      auto acts = ReceiveStructArray<AxisAct>(arr, px.GetAxisCount());
      px.Update(acts);
      ZeroCopySendStructArray(node, kOutputMotorCommands, px.GetCommands());

      if (px.IsEngaged() != was_engaged) {
        was_engaged = px.IsEngaged();
        if (was_engaged) {
          std::cout << "engaged (offset=" << px.GetOffset() << " rad)" << std::endl;
        } else if (px.IsEnabled()) {
          std::cout << "fault -> all OFF" << std::endl;
        }
      }
    }
  }
  return 0;
}
