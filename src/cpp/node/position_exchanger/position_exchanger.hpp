#pragma once
#include <cstddef>
#include <vector>
#include "../../lib/enum_def.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/robot_config.hpp"

// leader 軸の位置を follower 軸へ渡す (位置 PD 型インピーダンス制御)。
//   leader  : 電流制御 cur_d = cur_q = 0 (脱力。手で動かす側)
//   follower: POSITION_PD で leader の位置/速度に追従
//             FOCTIVE: cur_q = imp_kp*kp_scale*(pos_ref-pos) + imp_kd*kd_scale*(vel_ref-vel)
//             → トルク定数不要。imp_kp / imp_kd は FOCTIVE パラメータ側で設定すること。
//
// 起動直後は停止 (全軸 OFF)。SetEnabled(true) で開始し、その時点の両軸の位置差を
// オフセットとして保持する (相対写像。開始時に follower が跳ばない)。
// SetEnabled(false) またはどちらかの軸の fault (無応答 255 含む) で全軸 OFF。
// fault 復帰後は enable のままならオフセットを取り直して再開する。
class PositionExchanger {
 public:
  struct Params {
    int leader_device_id = 2;
    int follower_device_id = 1;
    double kp_scale = 1.0;
    double kd_scale = 1.0;
    // FOCTIVE は位置指令を台形プロファイルに通すため、大きめにして追従遅れを減らす
    double accel_limit = 1000.0;
  };

  // config から leader/follower の軸 index を引く。見つからなければ false。
  bool Configure(const RobotConfig& config, const Params& params);

  // 開始 / 停止。停止は即座に全軸 OFF。
  void SetEnabled(bool enabled);
  bool IsEnabled() const { return enabled_; }

  // motor_status 受信毎に呼ぶ。全軸分の指令を更新する。
  void Update(const std::vector<AxisAct>& status);

  const std::vector<AxisRef>& GetCommands() const { return commands_; }
  size_t GetAxisCount() const { return commands_.size(); }
  bool IsEngaged() const { return engaged_; }
  double GetOffset() const { return offset_; }

 private:
  void SetAllOff();

  Params params_;
  size_t leader_ = 0;
  size_t follower_ = 0;
  std::vector<AxisRef> commands_;
  bool enabled_ = false;
  bool engaged_ = false;
  double offset_ = 0.0;  // follower 位置 - leader 位置 (engage 時)
};
