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
#include <map>
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

// CAN チャンネル単位のドライバ束。ドライバは 1 インスタンス = 1 バス
// (SocketCanComm を値で保持) なので、comm_ch 毎にインスタンス化して束ねる。
// axes は config.axes の部分列で全体順序を保持する。ドライバ内部の
// expected_ids_ / last_frames_ は最初に渡した axes で確定するため、
// 各インスタンスには常に同じ部分列を渡すこと。
struct ChannelBundle {
    std::string netdev;
    std::unique_ptr<MotorDriver> driver;
    std::vector<AxisConfig> axes;       // このチャネル所属の軸
    std::vector<size_t> global_index;   // per-ch 位置 j → config.axes の index
    std::vector<AxisRef> cmd_buf;       // SendCommands 用の再利用バッファ
};

int main() {
    SetCpuAffinity(1, 80);
    auto node = init_dora_node();
    std::cout << "started" << std::endl;

    auto config = robot_config::LoadFromFile(ResolveConfigPath());
    std::cout << config.robot_name
              << " (" << config.axis_count << " axes)" << std::endl;

    // ドライバ初期化: comm_ch 毎に 1 インスタンス
    std::vector<ChannelBundle> channels(config.comm_ch.size());
    for (size_t ch = 0; ch < channels.size(); ++ch) {
        channels[ch].netdev = config.comm_ch[ch];
        channels[ch].driver = CreateDriver(config.transport, config.protocol);
    }
    for (size_t i = 0; i < config.axes.size(); ++i) {
        auto& b = channels[config.axes[i].comm_ch];  // 範囲は Parse で検証済み
        b.axes.push_back(config.axes[i]);
        b.global_index.push_back(i);
    }
    std::map<int, MotorDriver*> driver_by_device;  // settings dispatch 用
    for (auto& b : channels) {
        b.cmd_buf.resize(b.axes.size());
        if (b.axes.empty()) {
            std::cerr << "Warning: comm_ch " << b.netdev
                      << " has no axes" << std::endl;
        }
        for (const auto& ax : b.axes) {
            driver_by_device[ax.device_id] = b.driver.get();
        }
        std::string device =
            (config.transport == "dummy") ? "dummy" : b.netdev;
        if (!b.driver->Open(device)) {
            std::cerr << "failed to open " << device << std::endl;
            return 1;
        }
        std::cout << config.transport << " opened: " << device
                  << " (" << b.axes.size() << " axes)" << std::endl;
    }

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
            for (auto& b : channels) {
                if (!b.axes.empty()) b.driver->SendAllOff(b.axes);
            }
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
                // コマンド到達の確認ログ(送信時のみの低頻度なので常設)。
                // 分散構成で「送ったのに動かない」ときの切り分けに使う。
                std::cout << "recv motor_commands: state="
                          << static_cast<int>(latest_commands[0].motor_state)
                          << " ref=" << latest_commands[0].ref_val
                          << " ref1=" << latest_commands[0].ref_val_1 << std::endl;
            }
            else if (id == kInputTick) {
                auto t0 = std::chrono::steady_clock::now();

                // 送信は全チャネル先出し (バス往復をオーバーラップさせる)
                for (auto& b : channels) {
                    if (b.axes.empty()) continue;
                    if (has_new_commands) {
                        for (size_t j = 0; j < b.axes.size(); ++j) {
                            b.cmd_buf[j] = latest_commands[b.global_index[j]];
                        }
                        b.driver->SendCommands(b.cmd_buf, b.axes);
                    } else {
                        b.driver->SendQueries(b.axes);
                    }
                }
                has_new_commands = false;

                // 受信: 共有 2ms デッドライン + 各チャネル最低 1ms 保証。
                // ReceiveStatus は全 id 受信で早期リターンするため正常系の
                // コストは単一チャネル時と同等。未返信軸があるときのみ
                // 最悪 ~2+(N-1) ms まで伸びる。
                std::vector<AxisAct> acts(axis_count);  // 未返信軸はゼロのまま
                const auto rx_deadline = t0 + std::chrono::milliseconds(2);
                for (auto& b : channels) {
                    if (b.axes.empty()) continue;
                    long remaining_ms =
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            rx_deadline - std::chrono::steady_clock::now()).count();
                    if (remaining_ms < 1) remaining_ms = 1;
                    auto part = b.driver->ReceiveStatus(
                        b.axes, static_cast<int>(remaining_ms));
                    for (size_t j = 0; j < b.axes.size(); ++j) {
                        acts[b.global_index[j]] = part[j];  // 全体順に再構成
                    }
                }

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

                // device_id からチャンネルのドライバを解決
                auto it = driver_by_device.find(req.device_id);
                if (it == driver_by_device.end()) {
                    std::cerr << "settings_request: unknown device_id "
                              << req.device_id << std::endl;
                    res.ok = 0;
                    ZeroCopySendStruct(node, kOutputSettingsResult, res);
                    continue;
                }
                MotorDriver* driver = it->second;

                switch (req.cmd) {
                    case 1: {  // 電気角キャリブ(モータが回り数秒かかる)
                        // volt_d は req.value に float ビットで載っている
                        float volt_d;
                        std::memcpy(&volt_d, &req.value, 4);
                        float pos = 0;
                        bool ok = driver->Calibrate(
                            req.device_id, volt_d, &pos, 10000);
                        res.ok = ok ? 1 : 0;
                        std::memcpy(&res.value, &pos, 4);  // 完了時の機械角
                        break;
                    }
                    case 110: {  // 現在位置を任意の値として設定
                        // target_pos は req.value に float ビットで載っている
                        float target;
                        std::memcpy(&target, &req.value, 4);
                        float offset = 0;
                        bool ok = driver->AnyValPosOffset(
                            req.device_id, target, &offset, 50);
                        res.ok = ok ? 1 : 0;
                        std::memcpy(&res.value, &offset, 4);  // 適用 offset
                        break;
                    }
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
                    case 101: {  // 全パラメータ初期値ロード(返信は初期値, cmd=102と同形)
                        ParamScalars dump{};
                        bool ok = driver->LoadDefaultParams(
                            req.device_id, reinterpret_cast<uint8_t*>(&dump), 100);
                        res.ok = ok ? 1 : 0;
                        if (ok) ZeroCopySendStruct(node, kOutputParamDump, dump);
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
