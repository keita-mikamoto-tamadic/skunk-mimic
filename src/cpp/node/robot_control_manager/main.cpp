#include <iostream>
#include <vector>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include "dora-node-api.h"
#include "robot_control_manager.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/moteus_fault.hpp"
#include "../../lib/dora_helpers.hpp"

// config ファイルパス（dora 実行ディレクトリ = プロジェクトルート基準）
constexpr const char* kConfigPath = "robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputStateCommand  = "state_command";
constexpr const char* kInputWatchdog      = "watchdog";
constexpr const char* kInputMotorStatus   = "motor_status";
constexpr const char* kInputRunCommand    = "run_command";
constexpr const char* kOutputMotorCommands  = "motor_commands";
constexpr const char* kOutputStateStatus    = "state_status";


static const char* StateName(State s) {
    switch (s) {
        case State::OFF:   return "OFF";
        case State::STOP:  return "STOP";
        case State::READY: return "READY";
        case State::RUN:   return "RUN";
    }
    return "?";
}

static void SetRtPriority(int32_t priority) {
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: Failed to set RT Process priority" << std::endl;
    }
}

int main() {
    SetRtPriority(79);
    auto node = init_dora_node();
    std::cout << "started" << std::endl;

    // 起動時に config ファイルを直接読み込む
    auto config = robot_config::LoadFromFile(kConfigPath);
    std::cout << "loaded config: "
              << config.robot_name << " ("
              << config.axis_count << " axes)" << std::endl;

    RobotControlManager sm;
    sm.Configure(config, makeMoteusFaultEvaluator(State::OFF));

    bool motor_status_received = false;
    auto last_motor_status_time = std::chrono::steady_clock::now();
    int motor_status_count = 0;

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

            auto import_result = arrow::ImportArray(&c_array, &c_schema);
            if (!import_result.ok()) continue;
            auto arr = std::static_pointer_cast<arrow::UInt8Array>(
                import_result.ValueOrDie());

            if (id == kInputStateCommand) {
                auto cmd = ReceiveValue<StateCommand>(arr);

                if (cmd == StateCommand::RUN && !sm.IsReadyComplete()) {
                    std::cout << "RUN rejected: READY not complete"
                              << std::endl;
                } else {
                    State old_state = sm.GetState();
                    sm.HandleStateCommand(cmd);
                    std::cout << "cmd=" << (int)arr->Value(0)
                              << ": " << StateName(old_state)
                              << " -> " << StateName(sm.GetState()) << std::endl;
                }

                // 起動シーケンス: SERVO_ON 直後に初回 motor_commands を出力
                // （DCM が motor_status を返す → 以降 motor_status 駆動ループ）
                if (sm.GetState() == State::STOP) {
                    sm.RobotController();
                    ZeroCopySendStructArray(node, kOutputMotorCommands, sm.GetCommands());
                    ZeroCopySendStruct(node, kOutputStateStatus, sm.GetState());
                }
            }
            else if (id == kInputMotorStatus) {
                auto t0 = std::chrono::steady_clock::now();
                motor_status_count++;
                if (motor_status_count > 100) {
                    long interval_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        t0 - last_motor_status_time).count();
                    if (interval_us > 10000) {
                        std::cerr << "motor_status gap: " << interval_us << "us" << std::endl;
                    }
                }
                last_motor_status_time = t0;
                auto acts = ReceiveStructArray<AxisAct>(arr, sm.GetAxisCount());
                sm.UpdateMotorStatus(acts);
                sm.RobotController();
                auto t1 = std::chrono::steady_clock::now();
                ZeroCopySendStructArray(node, kOutputMotorCommands, sm.GetCommands());
                auto t2 = std::chrono::steady_clock::now();
                ZeroCopySendStruct(node, kOutputStateStatus, sm.GetState());
                auto t3 = std::chrono::steady_clock::now();
                motor_status_received = true;
                long calc_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
                long send_cmd_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                long send_st_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
                long total_us = calc_us + send_cmd_us + send_st_us;
                if (total_us > 5000) {
                    std::cerr << "RCM slow: " << total_us << "us"
                              << " (calc=" << calc_us
                              << " send_cmd=" << send_cmd_us
                              << " send_st=" << send_st_us << ")" << std::endl;
                }
            }
            else if (id == kInputWatchdog) {
                // ウォッチドッグ: motor_status 途絶検出
                if (!motor_status_received && sm.GetState() != State::OFF) {
                    std::cerr << "watchdog: motor_status timeout -> OFF"
                              << std::endl;
                    sm.HandleStateCommand(StateCommand::SERVO_OFF);
                    ZeroCopySendStructArray(node, kOutputMotorCommands, sm.GetCommands());
                    ZeroCopySendStruct(node, kOutputStateStatus, sm.GetState());
                }
                motor_status_received = false;
            }
            else if (id == kInputRunCommand) {
                auto refs = ReceiveStructArray<AxisRef>(arr, sm.GetAxisCount());
                sm.UpdateRunCommand(refs);
            }
        }
    }

    return 0;
}
