#pragma once
// AUTO-GENERATED from src/data_format/sensor_data.json by tools/gen_data_format.py
// DO NOT EDIT. 再生成: python3 tools/gen_data_format.py
#include <cstdint>
#include <cstddef>
#include "enum_def.hpp"

// imu_node / mujoco_backend → stabilizer, data_viewer: IMU 姿勢・加速度・角速度
struct ImuData {
    double timestamp;  // 秒
    double ax;  // 加速度 X
    double ay;  // 加速度 Y
    double az;  // 加速度 Z
    double gx;  // 角速度 X
    double gy;  // 角速度 Y
    double gz;  // 角速度 Z
    double q0;  // クォータニオン w
    double q1;  // クォータニオン x
    double q2;  // クォータニオン y
    double q3;  // クォータニオン z
    double roll;  // rad (X軸回転)
    double pitch;  // rad (Y軸回転)
    double yaw;  // rad (Z軸回転)
};
static_assert(sizeof(ImuData) == 112, "ImuData size mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, timestamp) == 0, "ImuData.timestamp offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, ax) == 8, "ImuData.ax offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, ay) == 16, "ImuData.ay offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, az) == 24, "ImuData.az offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, gx) == 32, "ImuData.gx offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, gy) == 40, "ImuData.gy offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, gz) == 48, "ImuData.gz offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, q0) == 56, "ImuData.q0 offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, q1) == 64, "ImuData.q1 offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, q2) == 72, "ImuData.q2 offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, q3) == 80, "ImuData.q3 offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, roll) == 88, "ImuData.roll offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, pitch) == 96, "ImuData.pitch offset mismatch vs sensor_data.json");
static_assert(offsetof(ImuData, yaw) == 104, "ImuData.yaw offset mismatch vs sensor_data.json");

// device_control_manager → data_viewer: レイテンシ計測結果
struct LatencyData {
    double can_avg_us;  // CAN 送受信の平均レイテンシ (us)
    double can_max_us;  // CAN 送受信の最大レイテンシ (us)
    double ctrl_avg_us;  // 制御側の平均レイテンシ (us)
    double ctrl_max_us;  // 制御側の最大レイテンシ (us)
};
static_assert(sizeof(LatencyData) == 32, "LatencyData size mismatch vs sensor_data.json");
static_assert(offsetof(LatencyData, can_avg_us) == 0, "LatencyData.can_avg_us offset mismatch vs sensor_data.json");
static_assert(offsetof(LatencyData, can_max_us) == 8, "LatencyData.can_max_us offset mismatch vs sensor_data.json");
static_assert(offsetof(LatencyData, ctrl_avg_us) == 16, "LatencyData.ctrl_avg_us offset mismatch vs sensor_data.json");
static_assert(offsetof(LatencyData, ctrl_max_us) == 24, "LatencyData.ctrl_max_us offset mismatch vs sensor_data.json");

// stabilizer → data_viewer: EKF 推定状態
struct EstimatedState {
    double velocity;  // s' [m/s] ボディ前進速度
    double yaw;  // alpha [rad] ヨー角
    double yaw_rate;  // alpha' [rad/s] ヨーレート
};
static_assert(sizeof(EstimatedState) == 24, "EstimatedState size mismatch vs sensor_data.json");
static_assert(offsetof(EstimatedState, velocity) == 0, "EstimatedState.velocity offset mismatch vs sensor_data.json");
static_assert(offsetof(EstimatedState, yaw) == 8, "EstimatedState.yaw offset mismatch vs sensor_data.json");
static_assert(offsetof(EstimatedState, yaw_rate) == 16, "EstimatedState.yaw_rate offset mismatch vs sensor_data.json");

