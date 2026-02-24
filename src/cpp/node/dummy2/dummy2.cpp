#include <iostream>
#include "dora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>

int main() {
  auto node = init_dora_node();
  std::cout << "[cpp-dummy2] started" << std::endl;

  while (true) {
    auto event = node.events->next();
    auto type = event_type(event);

    if (type == DoraEventType::Stop ||
        type == DoraEventType::AllInputsClosed) {
      std::cout << "[cpp-dummy2] stopping" << std::endl;
      break;
    }

    if (type == DoraEventType::Input) {
      // Arrow C Data Interface で受信
      struct ArrowArray c_array;
      struct ArrowSchema c_schema;
      auto result = event_as_arrow_input(
          std::move(event),
          reinterpret_cast<uint8_t*>(&c_array),
          reinterpret_cast<uint8_t*>(&c_schema));

      if (!result.error.empty()) {
        std::cerr << "[cpp-dummy2] error: " << std::string(result.error) << std::endl;
        continue;
      }

      // Arrow 配列にインポート
      auto import_result = arrow::ImportArray(&c_array, &c_schema);
      if (!import_result.ok()) {
        std::cerr << "[cpp-dummy2] import error: " << import_result.status().ToString() << std::endl;
        continue;
      }
      auto array = import_result.ValueOrDie();

      std::cout << "[cpp-dummy2] array length: " << array->length() << std::endl;

      // Int32Array にキャストして値を読み取り
      auto int_array = std::static_pointer_cast<arrow::FloatArray>(array);
      for (int64_t i = 0; i < int_array->length(); i++) {
        std::cout << "[cpp-dummy2] value[" << i << "]: " << int_array->Value(i) << std::endl;
      }
    }
  }

  return 0;
}
