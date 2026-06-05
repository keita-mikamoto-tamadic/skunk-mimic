#pragma once
#include <cstdint>
#include <cstring>
#include "enum_def.hpp"
#include "foctive_param_data.hpp"  // ParamIndex / kParamNum / MotParam


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

  // 設定モード (message bit = 15) の sub-command (README 設定モード表に対応)
  enum class SettingsCmd : uint8_t {
    kCalibration      = 1,    // 電気角キャリブ
    kIdentElec        = 2,    // 電気的パラメータ自動推定 (未実装)
    kParamSaveAll     = 100,  // 全パラメータセーブ
    kParamLoadDefault = 101,  // 全パラメータ初期値で設定
    kParamReadAll     = 102,  // 全パラメータ現在値読み出し (マルチフレーム返信)
    kParamSet         = 103,  // 個別パラメータ設定
    kParamRead        = 104,  // 個別パラメータ読み出し
    kZeroPosOffset    = 110,  // 現在位置を任意の値として設定
    kError            = 255,  // エラー応答 (param_num 不一致 / LUT 書込禁止 等)
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

  // 設定モード返信のデコード結果(単フレーム)
  struct SettingsReply {
    SettingsCmd cmd        = SettingsCmd::kError;  // 返ってきた cmd
    ParamIndex  index      = kInvalid;             // cmd=103/104 で有効
    bool        ok         = false;                // 正常に処理できたか
    uint8_t     warning    = 0;                    // cmd=255 のとき
    uint8_t     value[4]   = {0};                  // 104: 読み値 / 103: new 値
    uint8_t     old_value[4] = {0};                // 103: old 値
    float       pos        = 0;                     // 1: キャリブ完了時の機械角
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
  
  // ---- 設定モード(message bit=15)送信フレーム ----
  // 共通のヘッダ(CAN ID と cmd バイト)をセットして size を 1 にする
  inline void StartSettingsFrame(SettingsCmd cmd, uint8_t device_id, CanFdFrame& out) {
    out.canid_ = MakeCanId(static_cast<uint8_t>(MsgBit::kSettings), device_id);
    out.data[0] = static_cast<uint8_t>(cmd);
    out.size = 1;
  }

  // cmd=1 電気角キャリブ要求: [cmd, float volt_d]
  // volt_d はロックベクトル印加電圧。モータが実際に回り、完了まで数秒かかる。
  inline void MakeCalibrate(uint8_t device_id, float volt_d, CanFdFrame& out) {
    StartSettingsFrame(SettingsCmd::kCalibration, device_id, out);
    std::memcpy(&out.data[out.size], &volt_d, 4);
    out.size += 4;
  }

  // cmd=110 現在位置設定要求: [cmd, float target_pos]
  // 現在の機械角を target_pos として読ませる(ファームが offset を計算・保持)。
  inline void MakeZeroPosOffset(uint8_t device_id, float target_pos, CanFdFrame& out) {
    StartSettingsFrame(SettingsCmd::kZeroPosOffset, device_id, out);
    std::memcpy(&out.data[out.size], &target_pos, 4);
    out.size += 4;
  }

  // cmd=104 個別パラメータ読み出し要求: [cmd, param_index]
  inline void MakeReadParam(uint8_t device_id, ParamIndex index, CanFdFrame& out) {
    StartSettingsFrame(SettingsCmd::kParamRead, device_id, out);
    out.data[out.size++] = static_cast<uint8_t>(index);
  }

  // cmd=102 全パラメータ現在値読み出し要求: [cmd, param_num]
  inline void MakeReadAllParams(uint8_t device_id, CanFdFrame& out) {
    StartSettingsFrame(SettingsCmd::kParamReadAll, device_id, out);
    out.data[out.size++] = kParamNum;
  }

  // cmd=100 全パラメータセーブ(EEPROM)要求: [cmd, param_num]
  inline void MakeSaveAll(uint8_t device_id, CanFdFrame& out) {
    StartSettingsFrame(SettingsCmd::kParamSaveAll, device_id, out);
    out.data[out.size++] = kParamNum;
  }

  // cmd=101 全パラメータ初期値ロード要求: [cmd] のみ。返信はマルチフレーム(初期値)
  inline void MakeLoadDefault(uint8_t device_id, CanFdFrame& out) {
    StartSettingsFrame(SettingsCmd::kParamLoadDefault, device_id, out);
  }

  // cmd=103 個別パラメータ設定要求: [cmd, param_index, 4byte data]
  // (LUT index=8 は不可。呼び出し側で弾く)
  inline void MakeWriteParam(uint8_t device_id, ParamIndex index,
                             const uint8_t* data4, CanFdFrame& out) {
    StartSettingsFrame(SettingsCmd::kParamSet, device_id, out);
    out.data[out.size++] = static_cast<uint8_t>(index);
    std::memcpy(&out.data[out.size], data4, 4);
    out.size += 4;
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

  // 設定モード返信(単フレーム)をデコードし、MotParam に反映する。
  //   cmd=104 scalar: [cmd, index, 4byte]        → param[index] に書込
  //   cmd=103 set:    [cmd, index, old4, new4]   → new 値を param[index] に反映
  //   cmd=255 error:  [cmd, warning]
  // マルチフレーム(cmd=102 全読み出し / cmd=104 の LUT index=8)は別途。
  inline SettingsReply ParseSettings(const CanFdFrame& frame, MotParam& param) {
    SettingsReply r;
    const uint8_t* data = frame.data;
    r.cmd = static_cast<SettingsCmd>(data[0]);

    switch (r.cmd) {
      case SettingsCmd::kError:           // 255
        r.warning = data[1];
        r.ok = false;
        break;

      case SettingsCmd::kCalibration:     // 1: [cmd, done, pos(float)] (done=1 で完了)
        r.ok = (data[1] != 0);
        std::memcpy(&r.pos, &data[2], 4);
        break;

      case SettingsCmd::kZeroPosOffset:   // 110: [cmd, offset(float)] (適用 offset)
        std::memcpy(r.value, &data[1], 4);
        r.ok = true;
        break;

      case SettingsCmd::kParamSaveAll:    // 100: [cmd, done] (done=1 で完了)
        r.ok = (data[1] != 0);
        break;

      case SettingsCmd::kParamRead: {     // 104: [cmd, index, 4byte]
        ParamIndex idx = static_cast<ParamIndex>(data[1]);
        r.index = idx;
        if (idx == kElecAngleOfs) { r.ok = false; break; }  // LUT はマルチフレーム
        std::memcpy(r.value, &data[2], 4);                  // 読み値
        WriteParam(param, idx, &data[2]);
        r.ok = true;
        break;
      }

      case SettingsCmd::kParamSet: {       // 103: [cmd, index, old4, new4]
        ParamIndex idx = static_cast<ParamIndex>(data[1]);
        r.index = idx;
        std::memcpy(r.old_value, &data[2], 4);  // old 値(offset 2)
        std::memcpy(r.value, &data[6], 4);      // new 値(offset 6)
        WriteParam(param, idx, &data[6]);        // MotParam は new で更新
        r.ok = true;
        break;
      }

      default:
        r.ok = false;
        break;
    }
    return r;
  }

}
