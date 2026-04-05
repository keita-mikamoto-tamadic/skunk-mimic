#pragma once
#include <vector>
#include <memory>
#include "../../lib/enum_def.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/robot_config.hpp"
#include "../../interface/controller.hpp"
#include "../../controller/body_state_ekf.hpp"

class RobotControlManager {
public:
    RobotControlManager();

    // 設定（RobotConfig から軸情報・補間時間を取得）
    void Configure(const RobotConfig& config,
                   FaultEvaluator fault_evaluator = nullptr,
                   double tick_sec = 0.003);

    // 入力処理
    void HandleStateCommand(StateCommand cmd);
    void UpdateMotorStatus(const std::vector<AxisAct>& status);
    void UpdateImuData(const ImuData& imu);
    void UpdateRunCommand(const std::vector<AxisRef>& run_command);

    // 出力取得
    State GetState() const;
    size_t GetAxisCount() const;
    const std::vector<AxisRef>& GetCommands() const;
    bool IsReadyComplete() const;
    EstimatedState GetEstimatedState() const;

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

    // RUN 用: 外部ノードからのコマンド
    std::vector<AxisRef> run_command_;
    bool run_command_received_;

    // Controller + EKF（stabilizer から統合）
    std::unique_ptr<Controller> controller_;
    BodyStateEkf ekf_;
    ImuData imu_data_ = {};
    size_t wheel_r_ = SIZE_MAX;
    size_t wheel_l_ = SIZE_MAX;

    static constexpr double kWheelRadius = 0.07795;
    static constexpr double kCoMHeight = 0.2484;
};
