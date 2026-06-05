#include "foctive_can_driver.hpp"
#include <chrono>
#include <cstring>
#include <cstdio>

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

bool FoctiveCanDriver::ReadParam(
        int device_id, int param_index, uint8_t* out_value4, int timeout_ms) {
  Foctive::ParamIndex index = static_cast<Foctive::ParamIndex>(param_index);

  // 1. cmd=104 個別読み出し要求を送信(設定モード = 標準ID)
  Foctive::CanFdFrame tx;
  Foctive::MakeReadParam(static_cast<uint8_t>(device_id), index, tx);
  comm_.SendFrame(tx.canid_, tx.data, tx.size, /*extended=*/false);

  // 2. 同 device の設定モード返信を待つ
  uint8_t rx[kMaxFrameSize];
  size_t rxlen;
  auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

  while (true) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;
    int remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - now).count();
    if (remaining_ms <= 0) break;

    uint32_t can_id;
    if (!comm_.ReceiveAnyFrame(&can_id, rx, &rxlen, remaining_ms)) break;

    // device / 設定モード でフィルタ(返信は return bit=1, message=15)
    if (Foctive::ParseDevId(static_cast<uint16_t>(can_id)) != device_id) continue;
    if (Foctive::ParseMsgBit(static_cast<uint16_t>(can_id)) != Foctive::MsgBit::kSettings)
      continue;

    // 3. デコード → 保持 MotParam に反映
    Foctive::CanFdFrame frame;
    frame.canid_ = static_cast<uint16_t>(can_id);
    std::memcpy(frame.data, rx, rxlen);
    frame.size = static_cast<uint8_t>(rxlen);
    Foctive::SettingsReply reply = Foctive::ParseSettings(frame, params_[device_id]);

    if (reply.ok && reply.index == index) {
      // 4. 反映した 4byte 値を out へコピー
      if (out_value4) {
        std::memcpy(out_value4, Foctive::ParamPtr(params_[device_id], index), 4);
      }
      return true;
    }
    return false;  // エラー応答 / index 不一致
  }
  return false;  // タイムアウト
}

const Foctive::MotParam& FoctiveCanDriver::Params(int device_id) {
  return params_[device_id];
}

// 保持中の MotParam の 26 scalar を stdout に表示(LUT は除外、型で float/uint32)
static void PrintParams(int device_id, const Foctive::MotParam& p) {
  std::printf("[foctive] device %d params:\n", device_id);
  for (uint8_t i = 0; i < Foctive::kParamNum; i++) {
    if (i == Foctive::kElecAngleOfs) continue;  // LUT はスキップ
    const Foctive::ParamDesc& d = Foctive::kParamDesc[i];
    const uint8_t* ptr = Foctive::ParamPtr(p, static_cast<Foctive::ParamIndex>(i));
    if (d.is_float) {
      float f;
      std::memcpy(&f, ptr, 4);
      std::printf("  [%2u] %g\n", i, f);
    } else {
      uint32_t u;
      std::memcpy(&u, ptr, 4);
      std::printf("  [%2u] %u\n", i, u);
    }
  }
  std::fflush(stdout);
}

bool FoctiveCanDriver::ReadAllParams(int device_id, uint8_t* out_dump,
                                     int timeout_ms) {
  // 1. cmd=102 全読み出し要求 [102, param_num]
  Foctive::CanFdFrame tx;
  Foctive::MakeReadAllParams(static_cast<uint8_t>(device_id), tx);
  comm_.SendFrame(tx.canid_, tx.data, tx.size, /*extended=*/false);

  // 2. マルチフレーム受信: 各フレーム [cmd, is_last_frame, payload chunk] を連結
  uint8_t buf[sizeof(Foctive::MotParam)];
  size_t total = 0;
  bool last = false;

  uint8_t rx[kMaxFrameSize];
  size_t rxlen;
  auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

  while (!last) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;
    int remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - now).count();
    if (remaining_ms <= 0) break;

    uint32_t can_id;
    if (!comm_.ReceiveAnyFrame(&can_id, rx, &rxlen, remaining_ms)) break;
    if (Foctive::ParseDevId(static_cast<uint16_t>(can_id)) != device_id) continue;
    if (Foctive::ParseMsgBit(static_cast<uint16_t>(can_id)) != Foctive::MsgBit::kSettings)
      continue;
    if (rxlen < 2) continue;  // [cmd, is_last_frame] 最低
    if (rx[0] != static_cast<uint8_t>(Foctive::SettingsCmd::kParamReadAll))
      return false;  // cmd=255 エラー等

    last = (rx[1] != 0);
    size_t chunk = rxlen - 2;
    // 最終フレームは CAN-FD パディングで rxlen が膨らむ(例: 52→64)ため、
    // 既知の総量(sizeof MotParam)で残り容量に cap してパディングを捨てる
    size_t room = sizeof(buf) - total;
    if (chunk > room) chunk = room;
    std::memcpy(buf + total, &rx[2], chunk);
    total += chunk;
  }

  if (last && total == sizeof(Foctive::MotParam)) {
    // 3. 連結バイト列 = MotParam メモリ並び なので直接 memcpy
    std::memcpy(&params_[device_id], buf, sizeof(Foctive::MotParam));
    PrintParams(device_id, params_[device_id]);  // DCM ログ確認用

    // 4. ParamScalars(out_dump)へ詰める: [0..103]=26 scalar(index順), [104..359]=LUT
    if (out_dump) {
      size_t off = 0;
      for (uint8_t i = 0; i < Foctive::kParamNum; i++) {
        if (i == Foctive::kElecAngleOfs) continue;  // scalar 部には LUT を含めない
        std::memcpy(&out_dump[off],
                    Foctive::ParamPtr(params_[device_id],
                                      static_cast<Foctive::ParamIndex>(i)), 4);
        off += 4;
      }
      // 末尾に LUT 256byte
      std::memcpy(&out_dump[off], params_[device_id].elec_angle_ofs,
                  sizeof(params_[device_id].elec_angle_ofs));
    }
    return true;
  }
  return false;
}

bool FoctiveCanDriver::WriteParam(int device_id, int param_index,
                                  const uint8_t* value4, uint8_t* out_old4,
                                  uint8_t* out_new4, int timeout_ms) {
  Foctive::ParamIndex index = static_cast<Foctive::ParamIndex>(param_index);

  // 1. cmd=103 個別設定要求を送信 [103, index, 4byte data]
  Foctive::CanFdFrame tx;
  Foctive::MakeWriteParam(static_cast<uint8_t>(device_id), index, value4, tx);
  comm_.SendFrame(tx.canid_, tx.data, tx.size, /*extended=*/false);

  // 2. 同 device の設定モード返信を待つ
  uint8_t rx[kMaxFrameSize];
  size_t rxlen;
  auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

  while (true) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;
    int remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - now).count();
    if (remaining_ms <= 0) break;

    uint32_t can_id;
    if (!comm_.ReceiveAnyFrame(&can_id, rx, &rxlen, remaining_ms)) break;
    if (Foctive::ParseDevId(static_cast<uint16_t>(can_id)) != device_id) continue;
    if (Foctive::ParseMsgBit(static_cast<uint16_t>(can_id)) != Foctive::MsgBit::kSettings)
      continue;

    // 3. デコード(cmd=103: new 値を MotParam に反映)→ new 値を返す
    Foctive::CanFdFrame frame;
    frame.canid_ = static_cast<uint16_t>(can_id);
    std::memcpy(frame.data, rx, rxlen);
    frame.size = static_cast<uint8_t>(rxlen);
    Foctive::SettingsReply reply = Foctive::ParseSettings(frame, params_[device_id]);

    if (reply.ok && reply.index == index) {
      if (out_old4) std::memcpy(out_old4, reply.old_value, 4);  // before
      if (out_new4) std::memcpy(out_new4, reply.value, 4);      // after
      return true;
    }
    return false;  // エラー応答 / index 不一致
  }
  return false;  // タイムアウト
}
