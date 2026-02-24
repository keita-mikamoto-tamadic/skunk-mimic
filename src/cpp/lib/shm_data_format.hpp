#pragma once
#include <cstdint>

// state_manage → motor_comm: 1軸分の指令値
struct AxisRef {
    uint8_t motor_state;   // MotorState enum 値 (OFF=0, STOP=1, POSITION=2, VELOCITY=3)
    double ref_val;        // 目標値: position(rad) or velocity(rad/s)
    double kp_scale;       // 位置ゲインスケール
    double kv_scale;       // 速度ゲインスケール (= moteus の kd_scale)
    double velocity_limit; // rad/s
    double accel_limit;    // rad/s²
    double torque_limit;   // Nm
};

// motor_comm → state_manage: 1軸分の現在値
struct AxisAct {
    double position;       // rad
    double velocity;       // rad/s
    double torque;         // Nm
    uint8_t fault;         // 0=正常, それ以外=異常
};