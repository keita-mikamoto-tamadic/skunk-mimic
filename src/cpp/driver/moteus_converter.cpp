#include "moteus_converter.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include "../lib/vendor/mjbots/moteus/moteus_protocol.h"
using namespace mjbots;

constexpr double kTwoPi = 2.0 * M_PI;

uint32_t MoteusConverter::GetArbId(int device_id) {
  // 返信フレーム要求のためreply requiredビット
  // 0x8000を付加する
  return 0x8000 | static_cast<uint32_t>(device_id);
}

size_t MoteusConverter::BuildCommandFrame(
        uint8_t* buf, const AxisRef& ref, int motdir) {
  moteus::CanData frame;
  moteus::WriteCanData writer(&frame);
  
  auto state = ref.motor_state;

  // rad --> rev 単位変換 (moteusはrev)
  double ref_rev = (ref.ref_val / kTwoPi) * motdir;
  double vlim_rev = ref.velocity_limit / kTwoPi;
  double alim_rev = ref.accel_limit / kTwoPi;

  if (state == MotorState::SET_POSITION) {
    // OutputExact: エンコーダ位置リセット（OFF 時のみ使用）
    moteus::OutputExact::Command cmd;
    cmd.position = ref_rev;
    moteus::OutputExact::Format fmt;
    moteus::OutputExact::Make(&writer, cmd, fmt);
  } else if (state == MotorState::OFF) {
    // StopMode = motor current cut
    moteus::StopMode::Command cmd;
    moteus::StopMode::Format fmt;
    moteus::StopMode::Make(&writer, cmd, fmt);
  } else {
    moteus::PositionMode::Command cmd;
    cmd.feedforward_torque = 0.0;
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    
    moteus::PositionMode::Format fmt;
    
    switch (state) {
      case MotorState::STOP:
        // NaN位置 = 現在位置を保持してブレーキ
        cmd.position = std::numeric_limits<double>::quiet_NaN();
        cmd.velocity = 0.0;
        cmd.maximum_torque = ref.torque_limit;
        cmd.kp_scale = ref.kp_scale;
        cmd.kd_scale = ref.kv_scale;
        cmd.velocity_limit = std::numeric_limits<double>::quiet_NaN();
        cmd.accel_limit = std::numeric_limits<double>::quiet_NaN();
        break;
        
      case MotorState::POSITION:
        // 位置制御
        cmd.position = ref_rev;
        cmd.velocity = 0.0;
        cmd.maximum_torque = ref.torque_limit;
        cmd.kp_scale = ref.kp_scale;
        cmd.kd_scale = ref.kv_scale;
        cmd.velocity_limit = vlim_rev;
        cmd.accel_limit = alim_rev;
        break;
        
      case MotorState::VELOCITY:
        // 速度制御
        cmd.position = std::numeric_limits<double>::quiet_NaN();
        cmd.velocity = ref_rev;
        cmd.maximum_torque = ref.torque_limit;
        cmd.kp_scale = ref.kp_scale;
        cmd.kd_scale = ref.kv_scale;
        cmd.velocity_limit = vlim_rev;
        cmd.accel_limit = alim_rev;
        // kp = 0を正確にするためfloatにする
        fmt.kp_scale = moteus::Resolution::kFloat;
        fmt.kd_scale = moteus::Resolution::kFloat;
        break;

      case MotorState::TORQUE:
        // トルク制御: kp=0, kv=0, feedforward_torqueでトルク指令
        cmd.position = std::numeric_limits<double>::quiet_NaN();
        cmd.velocity = 0.0;
        cmd.feedforward_torque = ref.ref_val;  // [Nm] 単位変換不要
        cmd.maximum_torque = ref.torque_limit;
        cmd.kp_scale = 0.0;
        cmd.kd_scale = 0.0;
        cmd.velocity_limit = std::numeric_limits<double>::quiet_NaN();
        cmd.accel_limit = std::numeric_limits<double>::quiet_NaN();
        fmt.kp_scale = moteus::Resolution::kFloat;
        fmt.kd_scale = moteus::Resolution::kFloat;
        break;

      default:
        break;
    }
    moteus::PositionMode::Make(&writer, cmd, fmt);
  }
  
  // 全モード共通処理
  // Queryを付加してレスポンス要求
  moteus::Query::Format qfmt;
  qfmt.fault = moteus::Resolution::kInt8;
  moteus::Query::Make(&writer, qfmt);
  
  std::memcpy(buf, frame.data, frame.size);
  return frame.size;
}

size_t MoteusConverter::BuildQueryFrame(uint8_t* buf) {
  moteus::CanData frame;
  moteus::WriteCanData writer(&frame);

  // Query のみ（コマンドなし）
  moteus::Query::Format qfmt;
  qfmt.fault = moteus::Resolution::kInt8;
  moteus::Query::Make(&writer, qfmt);

  std::memcpy(buf, frame.data, frame.size);
  return frame.size;
}

size_t MoteusConverter::BuildResetPosition(
        uint8_t* buf, double position_rad, int motdir) {
    // rad → rev 変換（方向あり）
    double position_rev = (position_rad / kTwoPi) * motdir;

    moteus::CanData frame;
    moteus::WriteCanData writer(&frame);

    // OutputExact: エンコーダ位置を直接セット
    moteus::OutputExact::Command cmd;
    cmd.position = position_rev;
    moteus::OutputExact::Format fmt;
    moteus::OutputExact::Make(&writer, cmd, fmt);

    std::memcpy(buf, frame.data, frame.size);
    return frame.size;
}

bool MoteusConverter::ParseResponse(
        const uint8_t* data, size_t len, AxisAct& act, int motdir) {
    act = {};  // 全フィールドゼロ初期化

    auto result = moteus::Query::Parse(data, len);

    // rev → rad 変換（方向あり）
    act.position = result.position * kTwoPi * motdir;
    act.velocity = result.velocity * kTwoPi * motdir;
    // torque: Nm 共通、変換不要
    act.torque = result.torque;
    act.fault = result.fault;

    return true;
}