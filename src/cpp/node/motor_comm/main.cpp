#include <iostream>
#include <vector>
#include <cstring>
#include "dora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include "../../lib/robot_config.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../driver/socket_can_comm.hpp"
#include "../../driver/moteus_converter.hpp"

// config ファイルパス（dora 実行ディレクトリ = プロジェクトルート基準）
constexpr const char* kConfigPath = "robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputMotorCommands = "motor_commands";
constexpr const char* kOutputMotorStatus  = "motor_status";

// CAN フレームバッファサイズ（CAN-FD 最大）
constexpr size_t kMaxFrameSize = 64;

// AxisAct[] を UInt8Array として送信
static void SendMotorStatus(
  DoraNode& node,
  const std::vector<AxisAct>& acts)
{
  const size_t total = acts.size() * sizeof(AxisAct);
  auto* bytes = reinterpret_cast<const uint8_t*>(acts.data());

  arrow::UInt8Builder builder;
  ARROW_UNUSED(builder.AppendValues(bytes, total));
  std::shared_ptr<arrow::Array> array;
  ARROW_UNUSED(builder.Finish(&array));

  struct ArrowArray c_array;
  struct ArrowSchema c_schema;
  if (!arrow::ExportArray(*array, &c_array, &c_schema).ok()) return;

  send_arrow_output(
      node.send_output, rust::String(kOutputMotorStatus),
      reinterpret_cast<uint8_t*>(&c_array),
      reinterpret_cast<uint8_t*>(&c_schema));
}

// UInt8Array → AxisRef[] デシリアライズ
static std::vector<AxisRef> DeserializeMotorCommands(
  const std::shared_ptr<arrow::UInt8Array>& arr, size_t axis_count)
{
  std::vector<AxisRef> refs(axis_count);
  const size_t expected = axis_count * sizeof(AxisRef);
  if (static_cast<size_t>(arr->length()) >= expected) {
      std::memcpy(refs.data(), arr->raw_values(), expected);
  }
  return refs;
}

int main() {
  auto node = init_dora_node();
  std::cout << "[motor_comm] started" << std::endl;

  // 起動時に config ファイルを直接読み込む
  auto config = robot_config::LoadFromFile(kConfigPath);
  std::cout << "[motor_comm] loaded config: "
            << config.robot_name << " ("
            << config.axis_count << " axes)" << std::endl;

  // CAN 通信初期化
  SocketCanComm can;
  if (!can.Open("can0")) {
      std::cerr << "[motor_comm] failed to open CAN" << std::endl;
      return 1;
  }
  std::cout << "[motor_comm] CAN opened" << std::endl;

  MoteusConverter converter;
  const size_t axis_count = config.axes.size();

  while (true) {
      auto event = node.events->next();
      auto type = event_type(event);

      if (type == DoraEventType::Stop ||
          type == DoraEventType::AllInputsClosed) {
          // 全軸 OFF 送信
          uint8_t buf[kMaxFrameSize];
          AxisRef off_ref = {};
          off_ref.motor_state = static_cast<uint8_t>(MotorState::OFF);
          for (size_t i = 0; i < axis_count; i++) {
              const auto& ax = config.axes[i];
              size_t len = converter.BuildCommandFrame(
                  buf, off_ref, ax.motdir);
              can.SendFrame(
                  converter.GetArbId(ax.device_id), buf, len);
          }
          std::cout << "[motor_comm] stopping (all axes OFF)" << std::endl;
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

          if (id == kInputMotorCommands) {
              // motor_commands → 全軸送受信 → motor_status 送信
              auto refs = DeserializeMotorCommands(arr, axis_count);
              std::vector<AxisAct> acts(axis_count);
              uint8_t buf[kMaxFrameSize];
              uint8_t rx[kMaxFrameSize];
              size_t rxlen;

              for (size_t i = 0; i < axis_count; i++) {
                  const auto& ax = config.axes[i];
                  // コマンドフレーム送信
                  size_t len = converter.BuildCommandFrame(
                      buf, refs[i], ax.motdir);
                  can.SendFrame(
                      converter.GetArbId(ax.device_id), buf, len);
                  // レスポンス受信
                  if (can.ReceiveFrame(ax.device_id, rx, &rxlen, 3)) {
                      converter.ParseResponse(
                          rx, rxlen, acts[i], ax.motdir);
                  }
              }

              SendMotorStatus(node, acts);
          }
      }
  }

  return 0;
}