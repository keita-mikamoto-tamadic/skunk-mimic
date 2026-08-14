// src/cpp/lib/robot_config_test.cpp
#include "robot_config.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

namespace {

// 2チャンネル明示 config のひな型 (comm_ch テスト用)
std::string TwoChannelJson(const std::string& comm_ch,
                           int ax0_ch, int ax0_id,
                           int ax1_ch, int ax1_id) {
    return R"({
      "robot_name": "test",
      "axis_count": 2,
      "interpolation_time": 3.0,
      "comm_ch": )" + comm_ch + R"(,
      "axes": [
        {"index": 0, "name": "a0", "device_id": )" + std::to_string(ax0_id) +
        R"(, "comm_ch": )" + std::to_string(ax0_ch) + R"(, "motdir": 1,
         "initial_position": 0.0, "reset_position": 0.0,
         "velocity_limit": 1.0, "accel_limit": 1.0, "torque_limit": 1.0},
        {"index": 1, "name": "a1", "device_id": )" + std::to_string(ax1_id) +
        R"(, "comm_ch": )" + std::to_string(ax1_ch) + R"(, "motdir": 1,
         "initial_position": 0.0, "reset_position": 0.0,
         "velocity_limit": 1.0, "accel_limit": 1.0, "torque_limit": 1.0}
      ]
    })";
}

bool ParseThrows(const std::string& json) {
    try {
        robot_config::Parse(json);
        return false;
    } catch (const std::runtime_error&) {
        return true;
    }
}

} // anonymous namespace

int main() {
    // mimic_v2.json をパス指定で読み込み
    auto config = robot_config::LoadFromFile("../../../robot_config/mimic_v2.json");

    // --- トップレベル ---
    assert(config.robot_name == "mimic_v2");
    assert(config.axis_count == 6);
    assert(config.interpolation_time == 3.0);
    assert(config.axes.size() == 6);

    // --- comm_ch デフォルト (フィールド無し → {"can0"} / 全軸 0) ---
    assert(config.comm_ch.size() == 1);
    assert(config.comm_ch[0] == "can0");
    for (const auto& ax : config.axes) {
        assert(ax.comm_ch == 0);
    }

    // --- 軸0: hip_pitch_r（数値 limit） ---
    const auto& a0 = config.axes[0];
    assert(a0.index == 0);
    assert(a0.name == "hip_pitch_r");
    assert(a0.device_id == 50);
    assert(a0.motdir == 1);
    assert(a0.initial_position == 1.2);
    assert(a0.reset_position == -0.130899);
    assert(a0.velocity_limit == 100.0);
    assert(a0.accel_limit == 20.0);
    assert(a0.torque_limit == 15.0);

    // --- 軸2: wheel_r（"nan" limit） ---
    const auto& a2 = config.axes[2];
    assert(a2.index == 2);
    assert(a2.name == "wheel_r");
    assert(a2.device_id == 70);
    assert(std::isnan(a2.velocity_limit));  // "nan" → NaN
    assert(std::isnan(a2.accel_limit));     // "nan" → NaN
    assert(a2.torque_limit == 14.0);

    // --- 全軸ダンプ ---
    std::cout << "robot_name: " << config.robot_name << "\n";
    std::cout << "axis_count: " << config.axis_count << "\n";
    std::cout << "interpolation_time: " << config.interpolation_time << "\n";
    for (const auto& ax : config.axes) {
        std::cout << "  [" << ax.index << "] " << ax.name
                  << "  dev=" << ax.device_id
                  << "  motdir=" << ax.motdir
                  << "  init=" << ax.initial_position
                  << "  reset=" << ax.reset_position
                  << "  vlim=" << ax.velocity_limit
                  << "  alim=" << ax.accel_limit
                  << "  tlim=" << ax.torque_limit << "\n";
    }

    // --- comm_ch: 2チャンネル明示 config の正常パース ---
    {
        auto c = robot_config::Parse(
            TwoChannelJson(R"(["can0", "can1"])", 0, 50, 1, 80));
        assert(c.comm_ch.size() == 2);
        assert(c.comm_ch[0] == "can0");
        assert(c.comm_ch[1] == "can1");
        assert(c.axes[0].comm_ch == 0);
        assert(c.axes[1].comm_ch == 1);
    }

    // --- comm_ch: バリデーションエラー ---
    // インデックス範囲外
    assert(ParseThrows(TwoChannelJson(R"(["can0"])", 0, 50, 1, 80)));
    assert(ParseThrows(TwoChannelJson(R"(["can0", "can1"])", 0, 50, -1, 80)));
    // device_id 重複 (チャンネルが違っても不可)
    assert(ParseThrows(TwoChannelJson(R"(["can0", "can1"])", 0, 50, 1, 50)));
    // comm_ch 空配列
    assert(ParseThrows(TwoChannelJson(R"([])", 0, 50, 0, 80)));
    // netdev 名重複
    assert(ParseThrows(TwoChannelJson(R"(["can0", "can0"])", 0, 50, 1, 80)));

    std::cout << "\nAll assertions passed!\n";
    return 0;
}