#pragma once
#include <cstdint>
#include <optional>
#include <functional>

// モータ制御モード (通信先非依存)
enum class MotorState : uint8_t {
    OFF = 0,           // サーボ無効
    STOP = 1,          // 現在位置保持
    POSITION = 2,      // 位置制御
    VELOCITY = 3,      // 速度制御
    TORQUE = 4,        // トルク制御
    SET_POSITION = 5   // エンコーダ位置リセット（OFF時のみ）
};

// ステートマシンの状態
enum class State : uint8_t {
    OFF = 0,
    STOP = 1,
    READY = 2,
    RUN = 3
};

// 状態遷移コマンド (ref/ と同じ番号体系)
enum class StateCommand : uint8_t {
    STOP = 0,
    RUN = 1,
    SERVO_OFF = 2,
    SERVO_ON = 3,
    INIT_POSITION_RESET = 4,
    READY = 5
};

// フォルト判定関数型 (外部から注入)
// fault コードを受け取り、遷移先を返す。遷移不要なら nullopt。
using FaultEvaluator = std::function<std::optional<State>(uint8_t fault)>;

// 各軸のステータス
struct MotorStatus {
    double position;
    double velocity;
    double torque;
    uint8_t mode;
    uint8_t fault;
};