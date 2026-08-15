// src/cpp/node/imu_node/main.cpp
#include "dora-node-api.h"
#include "../../driver/spresense_imu.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/dora_helpers.hpp"
#include <iostream>

static constexpr const char* kOutputImuData = "raw_imu";

int main() {
    // 取り付け回転を config (imu_mount_rpy_deg) から取得。
    // config が読めなくても IMU 自体は動かす (恒等のまま warning のみ)
    std::array<double, 3> mount_rpy{0.0, 0.0, 0.0};
    try {
        auto config = robot_config::LoadFromFile(robot_config::ResolveConfigPath());
        mount_rpy = config.imu_mount_rpy_deg;
        std::cout << config.robot_name << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Warning: config load failed (" << e.what()
                  << "), imu mount = identity" << std::endl;
    }

    auto node = init_dora_node();
    std::cout << "started" << std::endl;

    // IMU 初期化
    SpresenseImu imu("/dev/ttyUSB0", 921600);
    imu.SetMountRotation(mount_rpy[0], mount_rpy[1], mount_rpy[2]);
    if (!imu.Open("/dev/ttyUSB0")) {
        std::cerr << "Failed to open IMU, continuing without sensor" << std::endl;
    } else {
        std::cout << "IMU initialized successfully" << std::endl;
    }

    // イベントループ
    while (true) {
        auto event = node.events->next();
        auto type = event_type(event);

        if (type == DoraEventType::Stop ||
            type == DoraEventType::AllInputsClosed) {
            std::cout << "stopping" << std::endl;
            break;
        }

        if (type == DoraEventType::Input) {
            struct ArrowArray c_array;
            struct ArrowSchema c_schema;
            auto info = event_as_arrow_input_with_info(
                std::move(event),
                reinterpret_cast<uint8_t*>(&c_array),
                reinterpret_cast<uint8_t*>(&c_schema));
            std::string id(info.id);

            if (id == "tick") {
                // 最新の IMU データを取得して送信
                ImuData data = imu.GetLatestData();
                ZeroCopySendStruct(node, kOutputImuData, data);
            }
        }
    }

    // クリーンアップ
    imu.Close();
    std::cout << "finished" << std::endl;
    return 0;
}
