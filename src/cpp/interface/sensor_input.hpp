// src/cpp/interface/sensor_input.hpp
#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

class SensorInput {
public:
    virtual ~SensorInput() = default;

    SensorInput(const SensorInput&) = delete;
    SensorInput& operator=(const SensorInput&) = delete;

    // -- Lifecycle --
    // device: センサー固有の識別子
    //   例: "/dev/ttyUSB0", "/dev/i2c-1", "tcp://192.168.1.100:8080"
    virtual bool Open(const std::string& device) = 0;
    virtual void Close() = 0;
    virtual bool IsOpen() const = 0;

    // -- Data acquisition --
    // data: 読み取ったデータを格納するバッファ
    // len: 入力時はバッファサイズ、出力時は実際に読み取ったバイト数
    // timeout_ms: タイムアウト時間（ミリ秒）
    virtual bool Read(uint8_t* data, size_t* len, int timeout_ms) = 0;

protected:
    SensorInput() = default;
};
