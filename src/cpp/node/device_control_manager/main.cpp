/**
 * device_control_manager: モーター通信 + motor_status/imu_data 出力
 *
 * 責務:
 *   - tick 駆動でモーター送受信 → motor_status 出力
 *   - motor_commands 受信 → コマンドバッファ更新
 *   - raw_imu 受信 → imu_data パススルー出力
 *   - SCHED_FIFO / CPU affinity 設定（リアルタイム）
 *
 * MotorDriver インターフェースで transport + protocol を切り替え:
 *   transport="dummy"                    → DummyDriver（テスト用）
 *   transport="socketcan", protocol="moteus"  → MoteusCanDriver（CAN-FD + moteus）
 *   transport="socketcan", protocol="foctive" → FoctiveCanDriver（CAN-FD + FOCTIVE）
 */
#include <iostream>
#include <memory>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include "dora-node-api.h"
#include <pthread.h>
#include <sched.h>

#include "../../lib/robot_config.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/dora_helpers.hpp"
#include "../../interface/motor_driver.hpp"
#include "../../driver/moteus_can_driver.hpp"
#include "../../driver/foctive_can_driver.hpp"
#include "../../driver/dummy_driver.hpp"

// config パスは環境変数 ROBOT_CONFIG で指定(未指定なら mimic_v2.json)
// dataflow yaml の env: ROBOT_CONFIG で切り替える
constexpr const char* kDefaultConfigPath = "robot_config/mimic_v2.json";

static std::string ResolveConfigPath() {
    const char* env = std::getenv("ROBOT_CONFIG");
    return (env && *env) ? env : kDefaultConfigPath;
}

// 入出力ID
constexpr const char* kInputTick          = "tick";
constexpr const char* kInputMotorCommands = "motor_commands";
constexpr const char* kInputImuData       = "raw_imu";
constexpr const char* kInputSettingsRequest = "settings_request";
constexpr const char* kOutputMotorStatus  = "motor_status";
constexpr const char* kOutputImuData      = "imu_data";
constexpr const char* kOutputLatency      = "latency";
constexpr const char* kOutputSettingsResult = "settings_result";
constexpr const char* kOutputParamDump      = "param_dump";

static void SetCpuAffinity(uint32_t core, int32_t priority) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);

    pthread_t current_thread = pthread_self();
    if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Warning: Failed to set CPU affinity" << std::endl;
    }

    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(current_thread, SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: Failed to set RT Process priority" << std::endl;
    }
}

static std::unique_ptr<MotorDriver> CreateDriver(const std::string& transport,
                                                 const std::string& protocol) {
    if (transport == "dummy") {
        return std::make_unique<DummyDriver>();
    }
    if (protocol == "foctive") {
        return std::make_unique<FoctiveCanDriver>();
    }
    return std::make_unique<MoteusCanDriver>();
}

int main() {
    SetCpuAffinity(1, 80);
    auto node = init_dora_node();
    std::cout << "started" << std::endl;

    auto config = robot_config::LoadFromFile(ResolveConfigPath());
    std::cout << config.robot_name
              << " (" << config.axis_count << " axes)" << std::endl;

    // ドライバ初期化
    auto driver = CreateDriver(config.transport, config.protocol);
    std::string device = (config.transport == "dummy") ? "dummy" : "can0";
    if (!driver->Open(device)) {
        std::cerr << "failed to open " << device << std::endl;
        return 1;
    }
    std::cout << config.transport << " opened" << std::endl;

    const size_t axis_count = config.axes.size();

    // コマンドバッファ
    std::vector<AxisRef> latest_commands(axis_count);
    bool has_new_commands = false;

    // レイテンシ計測
    int can_count = 0;
    long can_sum = 0;
    long can_max = 0;
    std::chrono::steady_clock::time_point status_send_time;
    bool status_pending = false;
    int ctrl_count = 0;
    long ctrl_sum = 0;
    long ctrl_max = 0;

    while (true) {
        auto event = node.events->next();
        auto type = event_type(event);

        if (type == DoraEventType::Stop ||
            type == DoraEventType::AllInputsClosed) {
            driver->SendAllOff(config.axes);
            std::cout << "stopping (all axes OFF)" << std::endl;
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

            // IMU パススルー（ゼロコピー）
            if (id == kInputImuData) {
                ForwardOutput(node, kOutputImuData, &c_array, &c_schema);
                continue;
            }

            // 以下は Arrow Import が必要
            auto import_result = arrow::ImportArray(&c_array, &c_schema);
            if (!import_result.ok()) continue;
            auto arr = std::static_pointer_cast<arrow::UInt8Array>(
                import_result.ValueOrDie());

            if (id == kInputMotorCommands) {
                // 制御側計測: motor_status 送信 → motor_commands 受信
                if (status_pending) {
                    long us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - status_send_time).count();
                    status_pending = false;
                    ctrl_sum += us;
                    if (us > ctrl_max) ctrl_max = us;
                    ctrl_count++;
                }
                latest_commands = ReceiveStructArray<AxisRef>(arr, axis_count);
                has_new_commands = true;
            }
            else if (id == kInputTick) {
                auto t0 = std::chrono::steady_clock::now();

                // 送受信
                if (has_new_commands) {
                    driver->SendCommands(latest_commands, config.axes);
                    has_new_commands = false;
                } else {
                    driver->SendQueries(config.axes);
                }
                auto acts = driver->ReceiveStatus(config.axes, 2);

                // CAN レイテンシ計測
                long us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - t0).count();
                can_sum += us;
                if (us > can_max) can_max = us;
                can_count++;

                // latency データ送信
                LatencyData latency;
                latency.can_avg_us = (can_count > 0) ? static_cast<double>(can_sum) / can_count : 0;
                latency.can_max_us = static_cast<double>(can_max);
                latency.ctrl_avg_us = (ctrl_count > 0) ? static_cast<double>(ctrl_sum) / ctrl_count : 0;
                latency.ctrl_max_us = static_cast<double>(ctrl_max);
                ZeroCopySendStruct(node, kOutputLatency, latency);

                ZeroCopySendStructArray(node, kOutputMotorStatus, acts);
                status_send_time = std::chrono::steady_clock::now();
                status_pending = true;
            }
            else if (id == kInputSettingsRequest) {
                // 設定モード要求(サーボOFF前提)。cmd で dispatch。
                SettingsRequest req = ReceiveStructArray<SettingsRequest>(arr, 1)[0];
                SettingsResult res{};
                res.cmd = req.cmd;
                res.param_index = req.param_index;

                switch (req.cmd) {
                    case 104: {  // 個別パラメータ読み出し
                        uint8_t val[4] = {0};
                        bool ok = driver->ReadParam(
                            req.device_id, req.param_index, val, 10);
                        res.ok = ok ? 1 : 0;
                        std::memcpy(&res.value, val, 4);
                        break;
                    }
                    case 102: {  // 全パラメータ読み出し(マルチフレーム)
                        // 26 scalar + LUT を ParamScalars(360byte)に受け取り param_dump で送る
                        ParamScalars dump{};
                        bool ok = driver->ReadAllParams(
                            req.device_id, reinterpret_cast<uint8_t*>(&dump), 100);
                        res.ok = ok ? 1 : 0;
                        if (ok) ZeroCopySendStruct(node, kOutputParamDump, dump);
                        break;
                    }
                    case 103: {  // 個別パラメータ設定(書き込み)
                        uint8_t oldval[4] = {0};
                        uint8_t newval[4] = {0};
                        bool ok = driver->WriteParam(
                            req.device_id, req.param_index,
                            reinterpret_cast<const uint8_t*>(&req.value),
                            oldval, newval, 10);
                        res.ok = ok ? 1 : 0;
                        std::memcpy(&res.value, newval, 4);      // after (new)
                        std::memcpy(&res.old_value, oldval, 4);  // before (old)
                        break;
                    }
                    case 100: {  // 全パラメータセーブ(EEPROM)
                        // EEPROM 書込で時間がかかるので長めの timeout
                        bool ok = driver->SaveAllParams(req.device_id, 200);
                        res.ok = ok ? 1 : 0;
                        break;
                    }
                    default:
                        res.ok = 0;  // 未対応 cmd
                        break;
                }
                ZeroCopySendStruct(node, kOutputSettingsResult, res);
            }
        }
    }

    return 0;
}
