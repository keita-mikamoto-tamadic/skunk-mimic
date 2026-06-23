#pragma once
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
};
static_assert(sizeof(AxisAct) == 48, "AxisAct size mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, position) == 0, "AxisAct.position offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, velocity) == 8, "AxisAct.velocity offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, torque) == 16, "AxisAct.torque offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, cur_d) == 24, "AxisAct.cur_d offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, cur_q) == 32, "AxisAct.cur_q offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, fault) == 40, "AxisAct.fault offset mismatch vs axis_data.json");

// foctive_controller → device_control_manager: 設定モード要求(cmd で分岐)
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

// device_control_manager → foctive_controller: 設定モード結果(scalar)
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

// device_control_manager → foctive_controller: cmd=102 全読み出し結果(26 scalar, 生ビット)。index は wire param_index(ParamIndex と一致)。LUT(elec_angle_ofs, index 8)は構造体上は末尾だが wire index は 8
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
    uint32_t vel_out_mx;
    uint32_t vel_out_mn;
    uint32_t pos_out_mx;
    uint32_t pos_out_mn;
    uint32_t elec_angle_ofs[64];  // 電気角オフセット LUT (wire index 8, 64要素)
};
static_assert(sizeof(ParamScalars) == 360, "ParamScalars size mismatch vs axis_data.json");
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
static_assert(offsetof(ParamScalars, vel_out_mx) == 88, "ParamScalars.vel_out_mx offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, vel_out_mn) == 92, "ParamScalars.vel_out_mn offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, pos_out_mx) == 96, "ParamScalars.pos_out_mx offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, pos_out_mn) == 100, "ParamScalars.pos_out_mn offset mismatch vs axis_data.json");
static_assert(offsetof(ParamScalars, elec_angle_ofs) == 104, "ParamScalars.elec_angle_ofs offset mismatch vs axis_data.json");

