#pragma once
#include <vector>
#include "../../lib/shm_data_format.hpp"
#include "../../lib/robot_config.hpp"
#include "body_state_ekf.hpp"

// 制御アルゴリズムの抽象基底クラス
// stabilizer main.cpp から呼ばれる共通インターフェース
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

    // 走行指令 (オペレータ入力)。使わないコントローラは無視してよいよう
    // デフォルト no-op (lqr は未対応のまま通る)。呼び出し側 (main) が
    // 途絶時に 0 の DriveCommand を渡す — 実装側で staleness は見なくてよい。
    virtual void SetDriveCommand(const DriveCommand& /*cmd*/) {}
};
