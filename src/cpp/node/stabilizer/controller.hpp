#pragma once
#include <vector>
#include "../../lib/shm_data_format.hpp"
#include "../../lib/robot_config.hpp"

// 制御アルゴリズムの抽象基底クラス
// stabilizer main.cpp から呼ばれる共通インターフェース
class Controller {
public:
    virtual ~Controller() = default;

    // RUN 遷移時のリセット（PID 積分値クリア等）
    virtual void Reset() = 0;

    // センサーデータ受信時の内部状態更新
    virtual void Update(const std::vector<AxisAct>& motor_status,
                        const ImuData& imu_data) = 0;

    // run_command を計算して返す（motor_status 駆動）
    virtual std::vector<AxisRef> Compute(const RobotConfig& config) = 0;

    // ボディ並進速度推定 [m/s]（具象クラスでオーバーライド）
    virtual double EstBodyVel() const { return 0.0; }
};
