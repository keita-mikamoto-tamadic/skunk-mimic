#include <iostream>
#include <vector>
#include "dora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>

int main() {
  // dora ノード初期化
  auto node = init_dora_node();
  std::cout << "[cpp_dummy] started" << std::endl;

  bool sending = false;
  int counter = 0;

  //イベントループ
  while (true) {
    auto event = node.events->next();
    auto type = event_type(event);

    if (type == DoraEventType::Stop ||
        type == DoraEventType::AllInputsClosed) {
      std::cout << "[cpp-dummy] stopping" << std::endl;
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
      
      if (id == "command") {
        auto import_result = arrow::ImportArray(&c_array,&c_schema);
        if (import_result.ok()) {
          auto str_array = std::static_pointer_cast<arrow::StringArray>(
            import_result.ValueOrDie());
          std::string cmd = str_array->GetString(0);
          if (cmd == "start") sending = true;
          else if (cmd == "stop") {
            counter = 0;
            sending = false;
          }
          std::cout<<"[cpp_dummy] command:" << cmd
                   <<" sending=" << (sending ? "true" : "false") << std::endl;
        }
        continue;
      }

      if (id == "tick" && sending) {
        // Arrow 配列をビルド
        std::vector<float> values = {counter * 1.1f, counter * 10.1f, counter * 100.1f};
        arrow::FloatBuilder builder;
        for (auto v : values) {
          ARROW_UNUSED(builder.Append(v));
        }
        std::shared_ptr<arrow::Array> array;
        ARROW_UNUSED(builder.Finish(&array));

        // Arrow C Data Interface にエクスポート
        struct ArrowArray c_array;
        struct ArrowSchema c_schema;
        auto export_status = arrow::ExportArray(*array, &c_array, &c_schema);
        if (!export_status.ok()) {
          std::cerr << "[cpp_dummy] export error: " << export_status.ToString() << std::endl;
          continue;
        }

        // send_arrow_output で送信
        auto result = send_arrow_output(
            node.send_output,
            rust::String("message"),
            reinterpret_cast<uint8_t*>(&c_array),
            reinterpret_cast<uint8_t*>(&c_schema));

        if (!result.error.empty()) {
          std::cerr << "[cpp_dummy] send error: " << std::string(result.error) << std::endl;
        }

        std::cout << "[cpp_dummy] sent message: [" << values[0] << ", "
                  << values[1] << ", " << values[2] << "]" << std::endl;

        counter++;
      }
    }
  }

  return 0;
}
