#pragma once

#include <cstdint>

namespace Foctive {
  enum ParamIndex : uint8_t {
    kMotorId       = 0,
    kDeviceId      = 1,
    kRotDir        = 2,
    kMechAngleDir  = 3,
    kElecAngleDir  = 4,
    kMotPolePairs  = 5,
    kGearEnable    = 6,
    kGearRatio     = 7,
    kZeroPosOfs    = 8,
    kPGainCur      = 9,
    kIGainCur      = 10,
    kDGainCur      = 11,
    kPGainVel      = 12,
    kIGainVel      = 13,
    kDGainVel      = 14,
    kPGainPos      = 15,
    kIGainPos      = 16,
    kDGainPos      = 17,
    kCurQMax       = 18,
    kCurQMin       = 19,
    kTrqOutMax     = 20,
    kTrqOutMin     = 21,
    kVelOutMax     = 22,
    kVelOutMin     = 23,
    kPosOutMax     = 24,
    kPosOutMin     = 25,
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
    float    zero_pos_ofs;
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
    float    vel_out_mx;
    float    vel_out_mn;
    float    pos_out_mx;
    float    pos_out_mn;
  };
};