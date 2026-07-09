#pragma once

#include <cstdint>
#include <cstddef>  // offsetof
#include <cstring>  // memcpy

namespace Foctive {
  // 全パラメータ個数 (= 29 scalar + 1 LUT block)。cmd=100/102 の param_num に使う
  constexpr uint8_t kParamNum = 30;

  enum ParamIndex : uint8_t {
    kMotorId       = 0,
    kDeviceId      = 1,
    kRotDir        = 2,
    kMechAngleDir  = 3,
    kElecAngleDir  = 4,
    kMotPolePairs  = 5,
    kGearEnable    = 6,
    kGearRatio     = 7,
    kElecAngleOfs  = 8,   // LUT (256 byte, uint32_t×64)。特殊扱い
    kAnyValPosOffset    = 9,
    kPGainCur      = 10,
    kIGainCur      = 11,
    kDGainCur      = 12,
    kPGainVel      = 13,
    kIGainVel      = 14,
    kDGainVel      = 15,
    kPGainPos      = 16,
    kIGainPos      = 17,
    kDGainPos      = 18,
    kCurQMax       = 19,
    kCurQMin       = 20,
    kTrqOutMax     = 21,
    kTrqOutMin     = 22,
    kVelocityLimit = 23,   // 台形プロファイル巡航速度制限 (旧 kVelOutMax)
    kAccelLimit    = 24,   // 台形プロファイル加速度制限 (旧 kVelOutMin)
    kPosOutMax     = 25,
    kPosOutMin     = 26,
    kImpKp         = 27,   // インピーダンス制御 位置ゲイン
    kImpKd         = 28,   // インピーダンス制御 速度ゲイン
    kTorqConst     = 29,   // モータ軸トルク定数 [Nm/A]
    kInvalid       = 0xFF,
  };

  struct MotParam{
    uint32_t motor_id;
    uint32_t device_id;
    uint32_t rot_dir;
    uint32_t mech_angle_dir;
    uint32_t elec_angle_dir;
    uint32_t mot_pole_pairs;
    uint32_t gear_enable;
    float    gear_ratio;
    uint32_t elec_angle_ofs[64];
    float    anyval_pos_offset;
    float    p_gain_cur;
    float    i_gain_cur;
    float    d_gain_cur;
    float    p_gain_vel;
    float    i_gain_vel;
    float    d_gain_vel;
    float    p_gain_pos;
    float    i_gain_pos;
    float    d_gain_pos;
    float    cur_q_mx;
    float    cur_q_mn;
    float    trq_out_mx;
    float    trq_out_mn;
    float    velocity_limit;   // 台形プロファイル巡航速度制限 (<=0 で無制限)
    float    accel_limit;      // 台形プロファイル加速度制限 (<=0 で無制限, RefVal 未指定時に使用)
    float    pos_out_mx;
    float    pos_out_mn;
    float    imp_kp;           // インピーダンス制御 位置ゲイン [A/rad]
    float    imp_kd;           // インピーダンス制御 速度ゲイン [A/(rad/s)]
    float    torq_const;       // モータ軸トルク定数 [Nm/A] (<=0 でトルク FF 無効)
  };

  // ParamIndex → MotParam 内のバイトオフセット / サイズ / 型。
  // LUT(index 8)が中央に挟まるので線形(index×4)にできない → offsetof で吸収。
  // MotParam のメモリ並び = README の index 表 = cmd=102 のワイヤ順、で一致している。
  struct ParamDesc {
    uint16_t offset;    // MotParam 先頭からのバイト位置
    uint16_t size;      // scalar=4, LUT=256
    bool     is_float;  // 表示・解釈用 (uint32 / LUT は false)
  };

  inline constexpr ParamDesc kParamDesc[kParamNum] = {
    {offsetof(MotParam, motor_id),       4,   false},  // 0
    {offsetof(MotParam, device_id),      4,   false},  // 1
    {offsetof(MotParam, rot_dir),        4,   false},  // 2
    {offsetof(MotParam, mech_angle_dir), 4,   false},  // 3
    {offsetof(MotParam, elec_angle_dir), 4,   false},  // 4
    {offsetof(MotParam, mot_pole_pairs), 4,   false},  // 5
    {offsetof(MotParam, gear_enable),    4,   false},  // 6
    {offsetof(MotParam, gear_ratio),     4,   true},   // 7
    {offsetof(MotParam, elec_angle_ofs), 256, false},  // 8  LUT
    {offsetof(MotParam, anyval_pos_offset),   4,   true},   // 9
    {offsetof(MotParam, p_gain_cur),     4,   true},   // 10
    {offsetof(MotParam, i_gain_cur),     4,   true},   // 11
    {offsetof(MotParam, d_gain_cur),     4,   true},   // 12
    {offsetof(MotParam, p_gain_vel),     4,   true},   // 13
    {offsetof(MotParam, i_gain_vel),     4,   true},   // 14
    {offsetof(MotParam, d_gain_vel),     4,   true},   // 15
    {offsetof(MotParam, p_gain_pos),     4,   true},   // 16
    {offsetof(MotParam, i_gain_pos),     4,   true},   // 17
    {offsetof(MotParam, d_gain_pos),     4,   true},   // 18
    {offsetof(MotParam, cur_q_mx),       4,   true},   // 19
    {offsetof(MotParam, cur_q_mn),       4,   true},   // 20
    {offsetof(MotParam, trq_out_mx),     4,   true},   // 21
    {offsetof(MotParam, trq_out_mn),     4,   true},   // 22
    {offsetof(MotParam, velocity_limit), 4,   true},   // 23
    {offsetof(MotParam, accel_limit),    4,   true},   // 24
    {offsetof(MotParam, pos_out_mx),     4,   true},   // 25
    {offsetof(MotParam, pos_out_mn),     4,   true},   // 26
    {offsetof(MotParam, imp_kp),         4,   true},   // 27
    {offsetof(MotParam, imp_kd),         4,   true},   // 28
    {offsetof(MotParam, torq_const),     4,   true},   // 29
  };

  // 受信した生バイト列を MotParam の該当フィールドへ書き込む(cmd=104/102 のデコード用)
  inline void WriteParam(MotParam& p, ParamIndex idx, const uint8_t* src) {
    const ParamDesc& d = kParamDesc[idx];
    std::memcpy(reinterpret_cast<uint8_t*>(&p) + d.offset, src, d.size);
  }

  // MotParam の該当フィールド先頭ポインタ(送信・表示用)
  inline const uint8_t* ParamPtr(const MotParam& p, ParamIndex idx) {
    return reinterpret_cast<const uint8_t*>(&p) + kParamDesc[idx].offset;
  }
};