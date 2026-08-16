#include <chrono>
#include <iostream>
#include <vector>
#include <cstdlib>
#include "dora-node-api.h"
#include "robot_control_manager.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/moteus_fault.hpp"
#include "../../lib/dora_helpers.hpp"

// RCM は RT 化しない (試して戻した): proc は安定するが wait (motor_status の
// 配送待ち 4〜7ms) は縮まず制御遅延は変わらない一方、RT 優先度が daemon の
// 配送スレッドを追い出して CAN 周期側を遅らせる要因にしかならないため。
//
// config パスは環境変数 ROBOT_CONFIG で指定(未指定なら mimic_v2.json)。
// 相対パスは robot_config::ResolveConfigPath がリポジトリルート基準で解決する。

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

int main() {
    auto node = init_dora_node();
    std::cout << "started" << std::endl;

    // 起動時に config ファイルを直接読み込む
    auto config = robot_config::LoadFromFile(robot_config::ResolveConfigPath());
    std::cout << "loaded config: "
              << config.robot_name << " ("
              << config.axis_count << " axes)" << std::endl;

    RobotControlManager sm;
    sm.Configure(config, makeMoteusFaultEvaluator(State::OFF));

    bool motor_status_received = false;

    // 受信レート計測 (診断用): motor_status を実際に何件/秒受け取れているかを
    // 1 秒ごとにログ出力する。DCM は 333Hz で送っているので、それを大きく
    // 下回れば daemon での配送落ち (queue 溢れ) が起きている。
    // 補間は motor_status の到着回数で進むため、落ちると補間が遅く/カクつく。
    // gap = 「前の motor_status 到着 → 次の到着」の最大。これを
    //   proc = 制御計算〜送信完了 (RCM 自身の処理時間) と
    //   wait = 送信完了 → 次イベント取り出し (next() で待っていた時間 = 配送 + スケジューリング)
    // に分解して、どちらが gap を作っているかを見る。
    long rx_count = 0;
    long rx_gap_max_us = 0;
    long proc_max_us = 0, proc_sum_us = 0;
    long wait_max_us = 0;
    auto rx_last = std::chrono::steady_clock::now();
    auto rx_window_start = rx_last;
    auto proc_end = rx_last;  // 直前の motor_status 処理が終わった時刻

    while (true) {
        auto event = node.events->next();
        auto type = event_type(event);
        auto t_recv = std::chrono::steady_clock::now();

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
                // 受信レート計測 (gap / wait)
                {
                    long gap = std::chrono::duration_cast<std::chrono::microseconds>(t_recv - rx_last).count();
                    long wait = std::chrono::duration_cast<std::chrono::microseconds>(t_recv - proc_end).count();
                    if (rx_count > 0) {
                        if (gap > rx_gap_max_us) rx_gap_max_us = gap;
                        if (wait > wait_max_us) wait_max_us = wait;
                    }
                    rx_last = t_recv;
                    ++rx_count;
                }
                // motor_status 駆動: ステータス更新 → 制御計算 → コマンド出力
                auto acts = ReceiveStructArray<AxisAct>(arr, sm.GetAxisCount());
                sm.UpdateMotorStatus(acts);
                sm.RobotController();
                ZeroCopySendStructArray(node, kOutputMotorCommands, sm.GetCommands());
                // 処理時間 (取り出し → motor_commands 送信完了)
                {
                    proc_end = std::chrono::steady_clock::now();
                    long proc = std::chrono::duration_cast<std::chrono::microseconds>(proc_end - t_recv).count();
                    proc_sum_us += proc;
                    if (proc > proc_max_us) proc_max_us = proc;
                    long win = std::chrono::duration_cast<std::chrono::milliseconds>(proc_end - rx_window_start).count();
                    if (win >= 1000) {
                        std::cout << "motor_status rx: " << rx_count << "/s"
                                  << "  gap max " << rx_gap_max_us << "us"
                                  << "  proc avg " << (rx_count ? proc_sum_us / rx_count : 0)
                                  << " max " << proc_max_us << "us"
                                  << "  wait max " << wait_max_us << "us" << std::endl;
                        rx_count = 0; rx_gap_max_us = 0; proc_max_us = 0; proc_sum_us = 0;
                        wait_max_us = 0; rx_window_start = proc_end;
                    }
                }
                // state_status は変化時 + キープアライブ (~10Hz) のみ。
                // 毎イベント (333Hz) 送ると、WiFi 越しの購読者 (robot_web_gui)
                // への転送が daemon を詰まらせ motor_status に 30ms 超の穴が開き、
                // 下の watchdog が誤発動する (実測 max 48ms)。
                static State last_sent_state = State::OFF;
                static int status_decim = 0;
                if (sm.GetState() != last_sent_state || ++status_decim >= 33) {
                    last_sent_state = sm.GetState();
                    status_decim = 0;
                    ZeroCopySendStruct(node, kOutputStateStatus, sm.GetState());
                }
                motor_status_received = true;
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
