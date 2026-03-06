#pragma once
#include <vector>
#include "../../lib/enum_def.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/pid.hpp"

class StateMachine {
public:
    StateMachine();

    // 設定（RobotConfig から軸情報・補間時間を取得）
    void Configure(const RobotConfig& config,
                   FaultEvaluator fault_evaluator = nullptr,
                   double tick_sec = 0.003);

    // 入力処理
    void HandleStateCommand(StateCommand cmd);
    void UpdateMotorStatus(const std::vector<AxisAct>& status);
    void UpdateImuData(double pitch, double pitch_rate);

    // 出力取得
    State GetState() const;
    size_t GetAxisCount() const;
    const std::vector<AxisRef>& GetCommands() const;
    bool IsReadyComplete() const;

    // tick ごとの制御出力 — ref_val を更新（motor_state は状態遷移時に設定済み）
    void RobotController();

private:
    State state_;
    RobotConfig config_;
    std::vector<AxisAct> axes_status_;
    FaultEvaluator fault_evaluator_;

    // 各軸のコマンド（メンバ変数として保持）
    std::vector<AxisRef> commands_;

    // 補間パラメータ
    double interp_progress_;
    double tick_sec_;
    std::vector<double> start_positions_;

    // エンコーダ位置リセット（1tick だけ SET_POSITION を出力）
    bool position_reset_pending_;

    // トルクリミット連続ヒットカウント（軸ごと）
    std::vector<int> torque_limit_count_;

    // IMU
    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;

    // PID（RUN 用）
    Pid angle_pid_;
    Pid velocity_pid_;
};
