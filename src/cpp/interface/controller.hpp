#pragma once
#include <vector>
#include "../lib/shm_data_format.hpp"
#include "../lib/robot_config.hpp"

class BodyStateEkf;  // 前方宣言

// 制御アルゴリズムの抽象基底クラス
class Controller {
public:
    virtual ~Controller() = default;

    // RUN 遷移時のリセット（PID 積分値クリア等）
    virtual void Reset() = 0;

    // センサーデータ + EKF推定値で内部状態更新
    virtual void Update(const std::vector<AxisAct>& motor_status,
                        const ImuData& imu_data,
                        const BodyStateEkf& ekf) = 0;

    // run_command を計算して返す
    virtual std::vector<AxisRef> Compute(const RobotConfig& config) = 0;
};
