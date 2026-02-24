#include <iostream>
#include <vector>
#include <cstring>
#include "dora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include "state_machine.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/moteus_fault.hpp"

// config ファイルパス（dora 実行ディレクトリ = プロジェクトルート基準）
constexpr const char* kConfigPath = "src/robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputStateCommand  = "state_command";
constexpr const char* kInputTick          = "tick";
constexpr const char* kInputMotorStatus   = "motor_status";
constexpr const char* kOutputMotorCommands  = "motor_commands";
constexpr const char* kOutputStateStatus    = "state_status";

// AxisRef[] を UInt8Array として送信
static void SendMotorCommands(
    DoraNode& node,
    const std::vector<AxisRef>& refs)
{
    const size_t total = refs.size() * sizeof(AxisRef);
    auto* bytes = reinterpret_cast<const uint8_t*>(refs.data());

    arrow::UInt8Builder builder;
    ARROW_UNUSED(builder.AppendValues(bytes, total));
    std::shared_ptr<arrow::Array> array;
    ARROW_UNUSED(builder.Finish(&array));

    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    if (!arrow::ExportArray(*array, &c_array, &c_schema).ok()) return;

    send_arrow_output(
        node.send_output, rust::String(kOutputMotorCommands),
        reinterpret_cast<uint8_t*>(&c_array),
        reinterpret_cast<uint8_t*>(&c_schema));
}

// state_status を UInt8Array (1要素) として送信
static void SendStateStatus(DoraNode& node, State state) {
    arrow::UInt8Builder builder;
    ARROW_UNUSED(builder.Append(static_cast<uint8_t>(state)));
    std::shared_ptr<arrow::Array> array;
    ARROW_UNUSED(builder.Finish(&array));

    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    if (!arrow::ExportArray(*array, &c_array, &c_schema).ok()) return;

    send_arrow_output(
        node.send_output, rust::String(kOutputStateStatus),
        reinterpret_cast<uint8_t*>(&c_array),
        reinterpret_cast<uint8_t*>(&c_schema));
}

// UInt8Array → AxisAct[] デシリアライズ
static std::vector<AxisAct> DeserializeMotorStatus(
    const std::shared_ptr<arrow::UInt8Array>& arr, size_t axis_count)
{
    std::vector<AxisAct> acts(axis_count);
    const size_t expected = axis_count * sizeof(AxisAct);
    if (static_cast<size_t>(arr->length()) >= expected) {
        std::memcpy(acts.data(), arr->raw_values(), expected);
    }
    return acts;
}

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
    sm.Configure(config, makeMoteusFaultEvaluator(State::STOP));
    ControlResult last_ctrl;

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
                auto cmd = static_cast<StateCommand>(arr->Value(0));

                if (cmd == StateCommand::RUN && !last_ctrl.ready_complete) {
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
                // tick 処理
                auto ctrl = sm.RobotController();
                SendMotorCommands(node, ctrl.commands);
                SendStateStatus(node, sm.GetState());
                last_ctrl = ctrl;
            }
            else if (id == kInputMotorStatus) {
                auto acts = DeserializeMotorStatus(arr, sm.GetAxisCount());
                sm.UpdateMotorStatus(acts);
            }
        }
    }

    return 0;
}