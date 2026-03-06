#include <iostream>
#include <vector>
#include "dora-node-api.h"
#include "state_machine.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/moteus_fault.hpp"
#include "../../lib/dora_helpers.hpp"

// config ファイルパス（dora 実行ディレクトリ = プロジェクトルート基準）
constexpr const char* kConfigPath = "robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputStateCommand  = "state_command";
constexpr const char* kInputTick          = "tick";
constexpr const char* kInputMotorStatus   = "motor_status";
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
    std::cout << "[state_manager] started" << std::endl;

    // 起動時に config ファイルを直接読み込む
    auto config = robot_config::LoadFromFile(kConfigPath);
    std::cout << "[state_manager] loaded config: "
              << config.robot_name << " ("
              << config.axis_count << " axes)" << std::endl;

    StateMachine sm;
    sm.Configure(config, makeMoteusFaultEvaluator(State::OFF));

    while (true) {
        auto event = node.events->next();
        auto type = event_type(event);

        if (type == DoraEventType::Stop ||
            type == DoraEventType::AllInputsClosed) {
            std::cout << "[state_manager] stopping" << std::endl;
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
                    std::cout << "[state_manager] RUN rejected: READY not complete"
                              << std::endl;
                } else {
                    State old_state = sm.GetState();
                    sm.HandleStateCommand(cmd);
                    std::cout << "[state_manager] cmd=" << (int)arr->Value(0)
                              << ": " << StateName(old_state)
                              << " -> " << StateName(sm.GetState()) << std::endl;
                }
            }
            else if (id == kInputTick) {
                sm.RobotController();
                SendStructArray(node, kOutputMotorCommands, sm.GetCommands());
                SendValue(node, kOutputStateStatus, sm.GetState());
            }
            else if (id == kInputMotorStatus) {
                auto acts = ReceiveStructArray<AxisAct>(arr, sm.GetAxisCount());
                sm.UpdateMotorStatus(acts);
            }
        }
    }

    return 0;
}
