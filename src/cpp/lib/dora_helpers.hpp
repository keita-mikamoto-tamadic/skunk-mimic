// src/cpp/lib/dora_helpers.hpp
#pragma once

#include "dora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <vector>
#include <cstring>
#include <memory>

// dora ノード用 Arrow データ送受信ヘルパー関数群

// 単一構造体を UInt8Array として送信
template<typename T>
void SendStruct(DoraNode& node, const char* output_id, const T& data)
{
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
    const size_t total = sizeof(T);

    arrow::UInt8Builder builder;
    ARROW_UNUSED(builder.AppendValues(bytes, total));
    std::shared_ptr<arrow::Array> array;
    ARROW_UNUSED(builder.Finish(&array));

    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    if (!arrow::ExportArray(*array, &c_array, &c_schema).ok()) return;

    send_arrow_output(
        node.send_output, rust::String(output_id),
        reinterpret_cast<uint8_t*>(&c_array),
        reinterpret_cast<uint8_t*>(&c_schema));
}

// 構造体配列を UInt8Array として送信
template<typename T>
void SendStructArray(DoraNode& node, const char* output_id, const std::vector<T>& data)
{
    const size_t total = data.size() * sizeof(T);
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data.data());

    arrow::UInt8Builder builder;
    ARROW_UNUSED(builder.AppendValues(bytes, total));
    std::shared_ptr<arrow::Array> array;
    ARROW_UNUSED(builder.Finish(&array));

    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    if (!arrow::ExportArray(*array, &c_array, &c_schema).ok()) return;

    send_arrow_output(
        node.send_output, rust::String(output_id),
        reinterpret_cast<uint8_t*>(&c_array),
        reinterpret_cast<uint8_t*>(&c_schema));
}

// 単一値（uint8 等）を送信
template<typename T>
void SendValue(DoraNode& node, const char* output_id, T value)
{
    arrow::UInt8Builder builder;
    ARROW_UNUSED(builder.Append(static_cast<uint8_t>(value)));
    std::shared_ptr<arrow::Array> array;
    ARROW_UNUSED(builder.Finish(&array));

    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    if (!arrow::ExportArray(*array, &c_array, &c_schema).ok()) return;

    send_arrow_output(
        node.send_output, rust::String(output_id),
        reinterpret_cast<uint8_t*>(&c_array),
        reinterpret_cast<uint8_t*>(&c_schema));
}

// UInt8Array から構造体配列を受信
template<typename T>
std::vector<T> ReceiveStructArray(const std::shared_ptr<arrow::UInt8Array>& arr, size_t count)
{
    std::vector<T> result(count);
    const size_t expected = count * sizeof(T);
    if (static_cast<size_t>(arr->length()) >= expected) {
        std::memcpy(result.data(), arr->raw_values(), expected);
    }
    return result;
}

// UInt8Array から単一値を受信
template<typename T>
T ReceiveValue(const std::shared_ptr<arrow::UInt8Array>& arr)
{
    return static_cast<T>(arr->Value(0));
}
