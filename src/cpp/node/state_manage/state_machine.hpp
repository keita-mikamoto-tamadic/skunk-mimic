#pragma once
#include <vector>
#include "../../lib/enum_def.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/robot_config.hpp"

struct ControlResult {
    std::vector<AxisRef> commands;
    bool ready_complete;
};

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

    // 出力取得
    State GetState() const;
    size_t GetAxisCount() const;

    // tick ごとの制御出力 — State に応じた AxisRef を生成
    // AxisRef の全フィールド（limits 含む）を config から埋める
    ControlResult RobotController();

private:
    State state_;
    RobotConfig config_;
    std::vector<AxisAct> axes_status_;
    FaultEvaluator fault_evaluator_;

    // 補間パラメータ
    double interp_progress_;
    double tick_sec_;
    std::vector<double> start_positions_;

    // エンコーダ位置リセット（1tick だけ SET_POSITION を出力）
    bool position_reset_pending_;
};