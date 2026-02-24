// src/cpp/lib/robot_config_test.cpp
#include "robot_config.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    // mimic_v2.json をパス指定で読み込み
    auto config = robot_config::LoadFromFile("../../../src/robot_config/mimic_v2.json");

    // --- トップレベル ---
    assert(config.robot_name == "mimic_v2");
    assert(config.axis_count == 6);
    assert(config.interpolation_time == 3.0);
    assert(config.axes.size() == 6);

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

    std::cout << "\nAll assertions passed!\n";
    return 0;
}