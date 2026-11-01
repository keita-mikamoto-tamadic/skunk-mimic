#pragma once
// Do not edit!
// AUTO-GENERATED from src/data_format/axis_data.json by tools/gen_data_format.py
// DO NOT EDIT. 再生成: python3 tools/gen_data_format.py
#include <cstdint>
#include <cstddef>
#include "enum_def.hpp"

// robot_control_manager → motor_comm: 1軸分の指令値
struct AxisRef {
    MotorState motor_state;  // 制御モード
    double ref_val;  // 第1値: pos/vel/torq, または volt_d/cur_d
    double ref_val_1;  // 第2値: volt_q/cur_q, またはインピーダンスvel
    double ref_val_2;  // 第3値: vir_ang_freq, またはインピーダンスtorq
    double kp_scale;  // 位置ゲインスケール
    double kv_scale;  // 速度ゲインスケール (= moteus の kd_scale)
    double velocity_limit;  // rad/s
    double accel_limit;  // rad/s^2
    double torque_limit;  // Nm
};
static_assert(sizeof(AxisRef) == 72, "AxisRef size mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, motor_state) == 0, "AxisRef.motor_state offset mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, ref_val) == 8, "AxisRef.ref_val offset mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, ref_val_1) == 16, "AxisRef.ref_val_1 offset mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, ref_val_2) == 24, "AxisRef.ref_val_2 offset mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, kp_scale) == 32, "AxisRef.kp_scale offset mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, kv_scale) == 40, "AxisRef.kv_scale offset mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, velocity_limit) == 48, "AxisRef.velocity_limit offset mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, accel_limit) == 56, "AxisRef.accel_limit offset mismatch vs axis_data.json");
static_assert(offsetof(AxisRef, torque_limit) == 64, "AxisRef.torque_limit offset mismatch vs axis_data.json");

// motor_comm → robot_control_manager: 1軸分の現在値
struct AxisAct {
    double position;  // rad
    double velocity;  // rad/s
    double torque;  // Nm
    double cur_d;  // d軸電流 A
    double cur_q;  // q軸電流 A
    uint8_t fault;  // 0=正常, それ以外=異常
    uint8_t mode;  // ドライバの動作モード (moteus: Mode enum。11=PositionTimeout, 0=Stopped 等)。未対応ドライバは 0
    uint8_t silent_ticks;  // この軸の無応答 tick の累積回数 (起動後、255 で飽和)。DCM が付与。散発的な取りこぼしの観測用
};
static_assert(sizeof(AxisAct) == 48, "AxisAct size mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, position) == 0, "AxisAct.position offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, velocity) == 8, "AxisAct.velocity offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, torque) == 16, "AxisAct.torque offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, cur_d) == 24, "AxisAct.cur_d offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, cur_q) == 32, "AxisAct.cur_q offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, fault) == 40, "AxisAct.fault offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, mode) == 41, "AxisAct.mode offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, silent_ticks) == 42, "AxisAct.silent_ticks offset mismatch vs axis_data.json");

// foctive_web_gui → device_control_manager: 設定モード要求(cmd で分岐)
struct SettingsRequest {
    uint8_t device_id;  // 対象デバイス
    uint8_t cmd;  // SettingsCmd (104=個別読出, 103=個別設定, 102=全読出, ...)
    uint8_t param_index;  // ParamIndex (cmd=103/104)
    uint32_t value;  // 4byte 生値(cmd=103 設定値。読出は未使用)
};
static_assert(sizeof(SettingsRequest) == 8, "SettingsRequest size mismatch vs axis_data.json");
static_assert(offsetof(SettingsRequest, device_id) == 0, "SettingsRequest.device_id offset mismatch vs axis_data.json");
static_assert(offsetof(SettingsRequest, cmd) == 1, "SettingsRequest.cmd offset mismatch vs axis_data.json");
static_assert(offsetof(SettingsRequest, param_index) == 2, "SettingsRequest.param_index offset mismatch vs axis_data.json");
static_assert(offsetof(SettingsRequest, value) == 4, "SettingsRequest.value offset mismatch vs axis_data.json");

// device_control_manager → foctive_web_gui: 設定モード結果(scalar)
struct SettingsResult {
    uint8_t cmd;  // 実行した SettingsCmd
    uint8_t param_index;  // ParamIndex
    uint8_t ok;  // 1=成功, 0=失敗/タイムアウト/非対応
    uint32_t value;  // 4byte 生値(104:読出値 / 103:new 値)
    uint32_t old_value;  // 4byte 生値(103:old 値。それ以外0)
};
static_assert(sizeof(SettingsResult) == 12, "SettingsResult size mismatch vs axis_data.json");
static_assert(offsetof(SettingsResult, cmd) == 0, "SettingsResult.cmd offset mismatch vs axis_data.json");
static_assert(offsetof(SettingsResult, param_index) == 1, "SettingsResult.param_index offset mismatch vs axis_data.json");
static_assert(offsetof(SettingsResult, ok) == 2, "SettingsResult.ok offset mismatch vs axis_data.json");
static_assert(offsetof(SettingsResult, value) == 4, "SettingsResult.value offset mismatch vs axis_data.json");
static_assert(offsetof(SettingsResult, old_value) == 8, "SettingsResult.old_value offset mismatch vs axis_data.json");

// device_control_manager → foctive_web_gui: cmd=102/101 全読み出し結果(47 scalar + LUT, 生ビット)。index は wire param_index(ParamIndex と一致)。LUT(elec_angle_ofs, index 8)は構造体上は末尾だが wire index は 8
struct ParamScalars {
    uint32_t motor_id;
    uint32_t device_id;
    uint32_t rot_dir;
    uint32_t mech_angle_dir;
    uint32_t elec_angle_dir;
    uint32_t mot_pole_pairs;
    uint32_t gear_enable;
    uint32_t gear_ratio;
    uint32_t anyval_pos_offset;
    uint32_t p_gain_cur;
    uint32_t i_gain_cur;
    uint32_t d_gain_cur;
    uint32_t p_gain_vel;
    uint32_t i_gain_vel;
    uint32_t d_gain_vel;
    uint32_t p_gain_pos;
    uint32_t i_gain_pos;
    uint32_t d_gain_pos;
    uint32_t cur_q_mx;
    uint32_t cur_q_mn;
    uint32_t trq_out_mx;
    uint32_t trq_out_mn;
    uint32_t velocity_limit;
    uint32_t accel_limit;
    uint32_t pos_out_mx;
    uint32_t pos_out_mn;
    uint32_t imp_kp;
    uint32_t imp_kd;
    uint32_t torq_const;
    uint32_t multiturn_bit;
    uint32_t multiturn_dir;
    uint32_t vernier_rev;
    uint32_t vernier_offset;
    uint32_t filter_reg1;
    uint32_t filter_reg2;
    uint32_t control_reg;
    uint32_t inertia;
    uint32_t static_friction;
    uint32_t viscous_friction;
    uint32_t back_emf_const;
    uint32_t phase_resistance;
    uint32_t phase_ld;
    uint32_t phase_lq;
    uint32_t free1;
    uint32_t free2;
    uint32_t free3;
    uint32_t free4;
    uint32_t coulomb_friction;
    uint32_t stribeck_velocity;
    uint32_t elec_angle_ofs[64];  // 電気角オフセット LUT (wire index 8, 64要素)
};
static_assert(sizeof(ParamScalars) == 452, "ParamScalars size mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, motor_id) == 0, "ParamScalars.motor_id offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, device_id) == 4, "ParamScalars.device_id offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, rot_dir) == 8, "ParamScalars.rot_dir offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, mech_angle_dir) == 12, "ParamScalars.mech_angle_dir offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, elec_angle_dir) == 16, "ParamScalars.elec_angle_dir offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, mot_pole_pairs) == 20, "ParamScalars.mot_pole_pairs offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, gear_enable) == 24, "ParamScalars.gear_enable offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, gear_ratio) == 28, "ParamScalars.gear_ratio offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, anyval_pos_offset) == 32, "ParamScalars.anyval_pos_offset offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, p_gain_cur) == 36, "ParamScalars.p_gain_cur offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, i_gain_cur) == 40, "ParamScalars.i_gain_cur offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, d_gain_cur) == 44, "ParamScalars.d_gain_cur offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, p_gain_vel) == 48, "ParamScalars.p_gain_vel offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, i_gain_vel) == 52, "ParamScalars.i_gain_vel offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, d_gain_vel) == 56, "ParamScalars.d_gain_vel offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, p_gain_pos) == 60, "ParamScalars.p_gain_pos offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, i_gain_pos) == 64, "ParamScalars.i_gain_pos offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, d_gain_pos) == 68, "ParamScalars.d_gain_pos offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, cur_q_mx) == 72, "ParamScalars.cur_q_mx offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, cur_q_mn) == 76, "ParamScalars.cur_q_mn offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, trq_out_mx) == 80, "ParamScalars.trq_out_mx offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, trq_out_mn) == 84, "ParamScalars.trq_out_mn offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, velocity_limit) == 88, "ParamScalars.velocity_limit offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, accel_limit) == 92, "ParamScalars.accel_limit offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, pos_out_mx) == 96, "ParamScalars.pos_out_mx offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, pos_out_mn) == 100, "ParamScalars.pos_out_mn offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, imp_kp) == 104, "ParamScalars.imp_kp offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, imp_kd) == 108, "ParamScalars.imp_kd offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, torq_const) == 112, "ParamScalars.torq_const offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, multiturn_bit) == 116, "ParamScalars.multiturn_bit offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, multiturn_dir) == 120, "ParamScalars.multiturn_dir offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, vernier_rev) == 124, "ParamScalars.vernier_rev offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, vernier_offset) == 128, "ParamScalars.vernier_offset offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, filter_reg1) == 132, "ParamScalars.filter_reg1 offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, filter_reg2) == 136, "ParamScalars.filter_reg2 offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, control_reg) == 140, "ParamScalars.control_reg offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, inertia) == 144, "ParamScalars.inertia offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, static_friction) == 148, "ParamScalars.static_friction offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, viscous_friction) == 152, "ParamScalars.viscous_friction offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, back_emf_const) == 156, "ParamScalars.back_emf_const offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, phase_resistance) == 160, "ParamScalars.phase_resistance offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, phase_ld) == 164, "ParamScalars.phase_ld offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, phase_lq) == 168, "ParamScalars.phase_lq offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, free1) == 172, "ParamScalars.free1 offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, free2) == 176, "ParamScalars.free2 offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, free3) == 180, "ParamScalars.free3 offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, free4) == 184, "ParamScalars.free4 offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, coulomb_friction) == 188, "ParamScalars.coulomb_friction offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, stribeck_velocity) == 192, "ParamScalars.stribeck_velocity offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, elec_angle_ofs) == 196, "ParamScalars.elec_angle_ofs offset mismatch vs axis_data.json");

// ParamScalars の wire index 順レイアウトと番号表 (相手側のメモリ並び)
namespace param_scalars {
struct Wire {
    uint32_t motor_id;
    uint32_t device_id;
    uint32_t rot_dir;
    uint32_t mech_angle_dir;
    uint32_t elec_angle_dir;
    uint32_t mot_pole_pairs;
    uint32_t gear_enable;
    float gear_ratio;
    uint32_t elec_angle_ofs[64];
    float anyval_pos_offset;
    float p_gain_cur;
    float i_gain_cur;
    float d_gain_cur;
    float p_gain_vel;
    float i_gain_vel;
    float d_gain_vel;
    float p_gain_pos;
    float i_gain_pos;
    float d_gain_pos;
    float cur_q_mx;
    float cur_q_mn;
    float trq_out_mx;
    float trq_out_mn;
    float velocity_limit;
    float accel_limit;
    float pos_out_mx;
    float pos_out_mn;
    float imp_kp;
    float imp_kd;
    float torq_const;
    uint32_t multiturn_bit;
    uint32_t multiturn_dir;
    float vernier_rev;
    float vernier_offset;
    uint32_t filter_reg1;
    uint32_t filter_reg2;
    uint32_t control_reg;
    float inertia;
    float static_friction;
    float viscous_friction;
    float back_emf_const;
    float phase_resistance;
    float phase_ld;
    float phase_lq;
    float free1;
    float free2;
    float free3;
    float free4;
    float coulomb_friction;
    float stribeck_velocity;
};
static_assert(sizeof(Wire) == 452, "ParamScalars wire size mismatch vs axis_data.json");
constexpr uint8_t kCount = 50;
enum Index : uint8_t {
    kMotorId = 0,
    kDeviceId = 1,
    kRotDir = 2,
    kMechAngleDir = 3,
    kElecAngleDir = 4,
    kMotPolePairs = 5,
    kGearEnable = 6,
    kGearRatio = 7,
    kElecAngleOfs = 8,
    kAnyvalPosOffset = 9,
    kPGainCur = 10,
    kIGainCur = 11,
    kDGainCur = 12,
    kPGainVel = 13,
    kIGainVel = 14,
    kDGainVel = 15,
    kPGainPos = 16,
    kIGainPos = 17,
    kDGainPos = 18,
    kCurQMx = 19,
    kCurQMn = 20,
    kTrqOutMx = 21,
    kTrqOutMn = 22,
    kVelocityLimit = 23,
    kAccelLimit = 24,
    kPosOutMx = 25,
    kPosOutMn = 26,
    kImpKp = 27,
    kImpKd = 28,
    kTorqConst = 29,
    kMultiturnBit = 30,
    kMultiturnDir = 31,
    kVernierRev = 32,
    kVernierOffset = 33,
    kFilterReg1 = 34,
    kFilterReg2 = 35,
    kControlReg = 36,
    kInertia = 37,
    kStaticFriction = 38,
    kViscousFriction = 39,
    kBackEmfConst = 40,
    kPhaseResistance = 41,
    kPhaseLd = 42,
    kPhaseLq = 43,
    kFree1 = 44,
    kFree2 = 45,
    kFree3 = 46,
    kFree4 = 47,
    kCoulombFriction = 48,
    kStribeckVelocity = 49,
    kInvalid = 0xFF,
};
struct Desc {
    uint16_t offset;    // Wire 先頭からのバイト位置
    uint16_t size;
    bool     is_float;  // repr == "float"
};
inline constexpr Desc kDesc[kCount] = {
    {0, 4, false},  // 0 motor_id
    {4, 4, false},  // 1 device_id
    {8, 4, false},  // 2 rot_dir
    {12, 4, false},  // 3 mech_angle_dir
    {16, 4, false},  // 4 elec_angle_dir
    {20, 4, false},  // 5 mot_pole_pairs
    {24, 4, false},  // 6 gear_enable
    {28, 4, true},  // 7 gear_ratio
    {32, 256, false},  // 8 elec_angle_ofs
    {288, 4, true},  // 9 anyval_pos_offset
    {292, 4, true},  // 10 p_gain_cur
    {296, 4, true},  // 11 i_gain_cur
    {300, 4, true},  // 12 d_gain_cur
    {304, 4, true},  // 13 p_gain_vel
    {308, 4, true},  // 14 i_gain_vel
    {312, 4, true},  // 15 d_gain_vel
    {316, 4, true},  // 16 p_gain_pos
    {320, 4, true},  // 17 i_gain_pos
    {324, 4, true},  // 18 d_gain_pos
    {328, 4, true},  // 19 cur_q_mx
    {332, 4, true},  // 20 cur_q_mn
    {336, 4, true},  // 21 trq_out_mx
    {340, 4, true},  // 22 trq_out_mn
    {344, 4, true},  // 23 velocity_limit
    {348, 4, true},  // 24 accel_limit
    {352, 4, true},  // 25 pos_out_mx
    {356, 4, true},  // 26 pos_out_mn
    {360, 4, true},  // 27 imp_kp
    {364, 4, true},  // 28 imp_kd
    {368, 4, true},  // 29 torq_const
    {372, 4, false},  // 30 multiturn_bit
    {376, 4, false},  // 31 multiturn_dir
    {380, 4, true},  // 32 vernier_rev
    {384, 4, true},  // 33 vernier_offset
    {388, 4, false},  // 34 filter_reg1
    {392, 4, false},  // 35 filter_reg2
    {396, 4, false},  // 36 control_reg
    {400, 4, true},  // 37 inertia
    {404, 4, true},  // 38 static_friction
    {408, 4, true},  // 39 viscous_friction
    {412, 4, true},  // 40 back_emf_const
    {416, 4, true},  // 41 phase_resistance
    {420, 4, true},  // 42 phase_ld
    {424, 4, true},  // 43 phase_lq
    {428, 4, true},  // 44 free1
    {432, 4, true},  // 45 free2
    {436, 4, true},  // 46 free3
    {440, 4, true},  // 47 free4
    {444, 4, true},  // 48 coulomb_friction
    {448, 4, true},  // 49 stribeck_velocity
};
}  // namespace param_scalars

