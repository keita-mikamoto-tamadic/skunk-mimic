#pragma once
#include <cstdint>
#include <optional>
#include <functional>
#include "enum_def.hpp"

enum class MoteusFault : uint8_t {
    // エラー (32-48): 即停止
    CalibrationFault     = 32,
    MotorDriverFault     = 33,
    Overvoltage          = 34,
    EncoderFault         = 35,
    MotorNotConfigured   = 36,
    PwmCycleOverrun      = 37,
    Overtemperature       = 38,
    OutsideLimit         = 39,
    Undervoltage         = 40,
    ConfigChanged        = 41,
    ThetaInvalid         = 42,
    PositionInvalid      = 43,
    DriverEnableFault    = 44,
    StopPositionDeprecated = 45,
    TimingViolation      = 46,
    BemfFeedforwardNoAccel = 47,
    InvalidLimits        = 48,
    // 警告 (96-103): 継続可能
    VelocityLimit        = 96,
    PowerLimit           = 97,
    SystemVoltageLimit   = 98,
    CurrentLimit         = 99,
    TemperatureLimit     = 100,
    MotorTemperatureLimit = 101,
    CommandedMaxTorque   = 102,
    PositionBoundsLimit  = 103,
};

// moteus 用フォルト判定関数を生成
// error_transition: エラー時の遷移先 (e.g. State::STOP)
// warning_transition: 警告時の遷移先 (nullopt なら無視)
inline FaultEvaluator makeMoteusFaultEvaluator(
    State error_transition,
    std::optional<State> warning_transition = std::nullopt)
{
    return [=](uint8_t fault) -> std::optional<State> {
        if (fault >= 32 && fault <= 48) return error_transition;
        if (warning_transition && fault >= 96 && fault <= 103) return *warning_transition;
        return std::nullopt;
    };
}