#pragma once
#include <cstdint>
#include "enum_def.hpp"

// robot_control_manager → motor_comm: 1軸分の指令値
struct AxisRef {
    MotorState motor_state;
    double ref_val;        // 目標値: position(rad) or velocity(rad/s)
    double kp_scale;       // 位置ゲインスケール
    double kv_scale;       // 速度ゲインスケール (= moteus の kd_scale)
    double velocity_limit; // rad/s
    double accel_limit;    // rad/s²
    double torque_limit;   // Nm
};

// motor_comm → robot_control_manager: 1軸分の現在値
struct AxisAct {
    double position;       // rad
    double velocity;       // rad/s
    double torque;         // Nm
    uint8_t fault;         // 0=正常, それ以外=異常
};

// device_control_manager → data_viewer: レイテンシ計測結果
struct LatencyData {
    double can_avg_us;     // CAN 送受信の平均レイテンシ (us)
    double can_max_us;     // CAN 送受信の最大レイテンシ (us)
    double ctrl_avg_us;    // 制御側の平均レイテンシ (us)
    double ctrl_max_us;    // 制御側の最大レイテンシ (us)
};

struct ImuData {
    double timestamp;
    double ax;
    double ay;
    double az;

    double gx;
    double gy;
    double gz;

    double q0;  // クォータニオン w
    double q1;  // クォータニオン x
    double q2;  // クォータニオン y
    double q3;  // クォータニオン z

    double roll;   // rad (X軸回転)
    double pitch;  // rad (Y軸回転)
    double yaw;    // rad (Z軸回転)
};