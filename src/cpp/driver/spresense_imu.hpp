// src/cpp/driver/spresense_imu.hpp
#pragma once
#include "../interface/imu_sensor.hpp"
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

class SpresenseImu : public ImuSensor {
public:
    // パケット定数
    static constexpr uint8_t kPacketHeader1 = 0xAA;
    static constexpr uint8_t kPacketHeader2 = 0x55;
    static constexpr size_t kPacketSize = 43;  // ヘッダ2 + float×10 + チェックサム1

    SpresenseImu(const std::string& port = "/dev/ttyUSB0", int baudrate = 921600);
    ~SpresenseImu() override;

    // -- SensorInput interface --
    bool Open(const std::string& device) override;
    void Close() override;
    bool IsOpen() const override { return serial_fd_ >= 0; }
    bool Read(uint8_t* data, size_t* len, int timeout_ms) override;

    // -- ImuSensor interface --
    ImuData GetLatestData() override;

private:
    void ReadThread();
    bool ValidateChecksum(const uint8_t* packet, size_t len);
    bool ParsePacket(const uint8_t* packet, size_t len);
    double GetNowSec();  // timestamp を秒で取得

    std::string port_;
    int baudrate_;
    int serial_fd_;

    std::thread read_thread_;
    std::atomic<bool> running_;

    std::mutex data_mutex_;
    ImuData latest_data_;

    std::vector<uint8_t> buffer_;
};
