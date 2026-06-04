#include "foctive_can_driver.hpp"
#include <chrono>
#include <cstring>

static constexpr size_t kMaxFrameSize = 64;

// AxisRef(アプリ指令) → Foctive::Command(プロトコル指令)
// 単位はそのまま(FOCTIVE は rad 系を想定)。機械方向は FOCTIVE 側の
// rot_dir パラメータで吸収するため、ここでは motdir を適用しない。
static Foctive::Command ToCommand(const AxisRef& ref) {
  Foctive::Command cmd;
  cmd.mode = ref.motor_state;
  switch (ref.motor_state) {
    case MotorState::VOLTAGE:
      cmd.volt_d       = ref.ref_val;
      cmd.volt_q       = ref.ref_val_1;
      cmd.vir_ang_freq = ref.ref_val_2;
      break;
    case MotorState::CURRENT:
      cmd.cur_d = ref.ref_val;
      cmd.cur_q = ref.ref_val_1;
      break;
    case MotorState::VELOCITY:
      cmd.vel = ref.ref_val;
      break;
    case MotorState::POSITION:
      cmd.pos = ref.ref_val;
      break;
    case MotorState::TORQUE:
      cmd.torq = ref.ref_val;
      break;
    case MotorState::POSITION_PD:  // インピーダンス制御
      cmd.pos      = ref.ref_val;
      cmd.vel      = ref.ref_val_1;
      cmd.torq     = ref.ref_val_2;
      cmd.kp_scale = ref.kp_scale;
      cmd.kd_scale = ref.kv_scale;
      break;
    case MotorState::STOP:
      // TODO: 現在位置保持。FOCTIVE にブレーキ保持モードが無いため暫定 idle(脱力)。
      //       将来は VELOCITY vel=0 か POSITION 現在位置保持に変更検討。
      cmd.mode = MotorState::OFF;
      break;
    default:  // SET_POSITION 等は idle 扱い
      cmd.mode = MotorState::OFF;
      break;
  }
  return cmd;
}

bool FoctiveCanDriver::Open(const std::string& device) {
  return comm_.Open(device);
}

void FoctiveCanDriver::Close() {
  comm_.Close();
}

void FoctiveCanDriver::SendCommands(const std::vector<AxisRef>& commands,
                                    const std::vector<AxisConfig>& axes) {
  if (last_frames_.size() != axes.size()) {
    last_frames_.resize(axes.size());
  }
  for (size_t i = 0; i < axes.size(); i++) {
    Foctive::Command cmd = ToCommand(commands[i]);
    // Make が CAN ID(message bit 含む)と data/size を組み立てる
    Foctive::Make(cmd, static_cast<uint8_t>(axes[i].device_id), last_frames_[i]);
    comm_.SendFrame(last_frames_[i].canid_, last_frames_[i].data,
                    last_frames_[i].size, /*extended=*/false);
  }
}

void FoctiveCanDriver::SendQueries(const std::vector<AxisConfig>& axes) {
  // FOCTIVE は純粋 Query フレームが無いため、直前コマンドを再送して返信を得る。
  if (last_frames_.size() != axes.size()) {
    // まだ一度も指令送信していない → OFF(idle)フレームで初期化
    last_frames_.resize(axes.size());
    Foctive::Command off;
    off.mode = MotorState::OFF;
    for (size_t i = 0; i < axes.size(); i++) {
      Foctive::Make(off, static_cast<uint8_t>(axes[i].device_id), last_frames_[i]);
    }
  }
  for (size_t i = 0; i < axes.size(); i++) {
    comm_.SendFrame(last_frames_[i].canid_, last_frames_[i].data,
                    last_frames_[i].size, /*extended=*/false);
  }
}

std::vector<AxisAct> FoctiveCanDriver::ReceiveStatus(
        const std::vector<AxisConfig>& axes, int timeout_ms) {
  // expected_ids を遅延構築
  if (expected_ids_.empty()) {
    for (const auto& ax : axes) {
      expected_ids_.insert(ax.device_id);
    }
  }

  std::vector<AxisAct> acts(axes.size());
  std::set<int> received_ids;
  uint8_t rx[kMaxFrameSize];
  size_t rxlen;

  auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

  while (received_ids.size() < expected_ids_.size()) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;

    int remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - now).count();
    if (remaining_ms <= 0) break;

    uint32_t can_id;
    if (comm_.ReceiveAnyFrame(&can_id, rx, &rxlen, remaining_ms)) {
      int device_id = Foctive::ParseDevId(static_cast<uint16_t>(can_id));
      if (expected_ids_.count(device_id) == 0) continue;

      for (size_t i = 0; i < axes.size(); i++) {
        if (axes[i].device_id == device_id) {
          // 受信バイト列を CanFdFrame に詰めて Parse
          Foctive::CanFdFrame frame;
          frame.canid_ = static_cast<uint16_t>(can_id);
          std::memcpy(frame.data, rx, rxlen);
          frame.size = static_cast<uint8_t>(rxlen);

          Foctive::Reply reply{};
          // 通常運用では設定モード以外の返信なので msg bit を渡してデコード
          Foctive::Parse(frame, reply, Foctive::ParseMsgBit(frame.canid_));

          acts[i].position = reply.pos;
          acts[i].velocity = reply.vel;
          acts[i].torque   = reply.torq;
          acts[i].fault    = reply.alarm;
          received_ids.insert(device_id);
          break;
        }
      }
    }
  }

  return acts;
}

void FoctiveCanDriver::SendAllOff(const std::vector<AxisConfig>& axes) {
  Foctive::Command off;
  off.mode = MotorState::OFF;
  Foctive::CanFdFrame frame;
  for (const auto& ax : axes) {
    Foctive::Make(off, static_cast<uint8_t>(ax.device_id), frame);
    comm_.SendFrame(frame.canid_, frame.data, frame.size, /*extended=*/false);
  }
}
