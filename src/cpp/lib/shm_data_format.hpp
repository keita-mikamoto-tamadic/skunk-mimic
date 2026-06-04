#pragma once
#include <cstdint>
#include "enum_def.hpp"

// AxisRef / AxisAct は src/data_format/axis_data.json を正本に自動生成
// (tools/gen_data_format.py / CMake configure 時に再生成)
#include "data_format_generated.hpp"

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

struct EstimatedState {
    double velocity;    // ṡ [m/s] ボディ前進速度
    double yaw;         // α [rad] ヨー角
    double yaw_rate;    // α̇ [rad/s] ヨーレート
};