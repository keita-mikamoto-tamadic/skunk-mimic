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
    uint8_t fault;  // 0=正常, それ以外=異常
};
static_assert(sizeof(AxisAct) == 32, "AxisAct size mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, position) == 0, "AxisAct.position offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, velocity) == 8, "AxisAct.velocity offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, torque) == 16, "AxisAct.torque offset mismatch vs axis_data.json");
static_assert(offsetof(AxisAct, fault) == 24, "AxisAct.fault offset mismatch vs axis_data.json");

