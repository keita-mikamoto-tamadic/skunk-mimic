#include "position_exchanger.hpp"

bool PositionExchanger::Configure(const RobotConfig& config, const Params& params) {
  params_ = params;
  bool found_leader = false;
  bool found_follower = false;
  for (size_t i = 0; i < config.axes.size(); ++i) {
    if (config.axes[i].device_id == params.leader_device_id) {
      leader_ = i;
      found_leader = true;
    }
    if (config.axes[i].device_id == params.follower_device_id) {
      follower_ = i;
      found_follower = true;
    }
  }
  commands_.assign(config.axes.size(), AxisRef{});
  SetAllOff();
  return found_leader && found_follower && leader_ != follower_;
}

void PositionExchanger::SetAllOff() {
  for (auto& c : commands_) {
    c = AxisRef{};
    c.motor_state = MotorState::OFF;
  }
  engaged_ = false;
}

void PositionExchanger::SetEnabled(bool enabled) {
  enabled_ = enabled;
  if (!enabled_) SetAllOff();
}

void PositionExchanger::Update(const std::vector<AxisAct>& status) {
  if (status.size() != commands_.size()) return;
  if (!enabled_) return;  // 停止中は OFF のまま
  const AxisAct& leader = status[leader_];
  const AxisAct& follower = status[follower_];

  if (leader.fault != 0 || follower.fault != 0) {
    SetAllOff();
    return;
  }

  if (!engaged_) {
    // OFF → 制御モードは直接遷移可 (FOCTIVE はモード間の直接遷移のみ禁止)
    offset_ = follower.position - leader.position;
    engaged_ = true;
  }

  AxisRef& l = commands_[leader_];
  l = AxisRef{};
  l.motor_state = MotorState::CURRENT;
  l.ref_val = 0.0;    // cur_d
  l.ref_val_1 = 0.0;  // cur_q
  l.kp_scale = 1.0;
  l.kv_scale = 1.0;

  AxisRef& f = commands_[follower_];
  f = AxisRef{};
  f.motor_state = MotorState::POSITION_PD;
  f.ref_val = leader.position + offset_;  // pos
  f.ref_val_1 = leader.velocity;          // vel
  f.ref_val_2 = 0.0;                      // FF トルク (torq_const 未設定なので使わない)
  f.kp_scale = params_.kp_scale;
  f.kv_scale = params_.kd_scale;
  f.accel_limit = params_.accel_limit;
}
