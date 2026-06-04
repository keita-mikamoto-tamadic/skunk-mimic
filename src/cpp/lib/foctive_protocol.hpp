#pragma once
#include <cstdint>
#include <cstring>
#include "enum_def.hpp"


namespace Foctive {

  enum class MsgBit : uint8_t {
    kIdle             = 0,
    kVoltageControl   = 1,
    kCurrentControl   = 2,
    kVelocityControl  = 3,
    kPositionControl  = 4,
    kTorqueFB         = 5,
    kImpedanceControl = 6,
    // 7~14: reserve
    kSettings         = 15,  // サーボ OFF
  };

  // 設定モード (message bit = 15) の sub-command
  enum class SettingsCmd : uint8_t {
    kCalibration   = 1,    // 電気角キャリブ
    kIdentElec     = 2,    // 電気的パラメータ自動推定 (未実装)
    kParamSave     = 100,  // 全パラメータセーブ
    kParamLoad     = 101,  // 全パラメータ初期値ロード
    kParamSet      = 102,  // 個別パラメータ設定
    kLutRead       = 103,  // 電気角オフセット LUT 読み出し
    kZeroPosOffset = 105,  // 現在位置で角度オフセット設定
  };

  struct CanFdFrame {
    uint16_t canid_ = 0;
    uint8_t data[64] = {0};
    uint8_t size = 0;
  };

  struct Command {
    MotorState mode = MotorState::OFF;
    float volt_d = 0, volt_q = 0, vir_ang_freq = 0;  // 電圧
    float cur_d = 0, cur_q = 0;                       // 電流
    float vel = 0, pos = 0, torq = 0;                 // 速度/位置/トルク
    float kp_scale = 1, kd_scale = 1;                 // インピーダンス
  };

  struct Reply {
    uint8_t mode;
    uint8_t alarm;
    uint8_t warning;
    float cur_d;
    float cur_q;
    float torq;
    float vel;
    float pos;
    float power_sup_volt;
    float mcu_temp;
  };

  // MotorState(名前) → FOCTIVE message bit(ワイヤ番号)
  inline MsgBit ToMessageBit(MotorState mode) {
    switch (mode) {
      case MotorState::OFF:         return MsgBit::kIdle;
      case MotorState::VOLTAGE:     return MsgBit::kVoltageControl;
      case MotorState::CURRENT:     return MsgBit::kCurrentControl;
      case MotorState::VELOCITY:    return MsgBit::kVelocityControl;
      case MotorState::POSITION:    return MsgBit::kPositionControl;
      case MotorState::TORQUE:      return MsgBit::kTorqueFB;
      case MotorState::POSITION_PD: return MsgBit::kImpedanceControl;
      default:                      return MsgBit::kIdle;
    }
  }

  // CAN ID 組み立て (11bit: [return(1) | message(4) | device(6)])
  // return bit: マスター送信 = 0
  inline uint16_t MakeCanId(uint8_t message_bit, uint8_t device_id) {
    return (uint16_t(message_bit & 0x0F) << 6) | (device_id & 0x3F);
  }
  
  // device idのみ取り出す
  inline uint8_t ParseDevId(uint16_t canid) {
    return canid & 0x3F;
  }
  
  // Mssage bitのみ取り出す
  inline MsgBit ParseMsgBit(uint16_t canid) {
    return static_cast<MsgBit>((canid >> 6) & 0x0F);
  }

  // Ref Command -> mode で switch -> CanFdFrame 組み立て
  inline void Make(const Command& cmd, uint8_t device_id, CanFdFrame& out) {
    out.canid_ = MakeCanId(static_cast<uint8_t>(ToMessageBit(cmd.mode)), device_id);
    out.size = 0;

    // float を Little Endian で末尾に追記 (memcpy で 4byte)
    // lamda式 [キャプチャリスト] 参照でMake引数のoutを取り込む.参照を外すとコピーが作成される.
    auto push = [&out](float v) {
      std::memcpy(&out.data[out.size], &v, sizeof(float));
      out.size += sizeof(float);
    };

    switch (cmd.mode) {
      case MotorState::VOLTAGE:
        push(cmd.volt_d);
        push(cmd.volt_q);
        push(cmd.vir_ang_freq);
        break;

      case MotorState::CURRENT:
        push(cmd.cur_d);
        push(cmd.cur_q);
        break;

      case MotorState::VELOCITY:
        push(cmd.vel);
        break;

      case MotorState::POSITION:
        push(cmd.pos);
        break;

      case MotorState::TORQUE:
        push(cmd.torq);
        break;

      case MotorState::POSITION_PD:  // インピーダンス制御
        push(cmd.pos);
        push(cmd.vel);
        push(cmd.torq);
        push(cmd.kp_scale);
        push(cmd.kd_scale);
        break;

      case MotorState::OFF:
      default:
        // アイドル: ペイロードなし
        break;
    }
  }
  
  inline void Parse(const CanFdFrame& frame, Reply& out, MsgBit return_msg) {
    uint8_t offset = 0;

    const uint8_t* data = frame.data;
    
    auto pull_uint8 = [&data, &offset]() {
      uint8_t v = data[offset]; offset += 1;
      return v;
    };
    auto pull_float = [&data, &offset]() {
      float v;
      std::memcpy(&v, &data[offset], sizeof(float));
      offset += sizeof(float);
      return v;
    };

    if (return_msg != MsgBit::kSettings) {
      // 標準返信
      out.mode = pull_uint8();
      out.alarm = pull_uint8();
      out.warning = pull_uint8();
      out.cur_d = pull_float();
      out.cur_q = pull_float();
      out.torq = pull_float();
      out.vel = pull_float();
      out.pos = pull_float();
      out.power_sup_volt = pull_float();
      out.mcu_temp = pull_float();
    }else{}
  }

}
