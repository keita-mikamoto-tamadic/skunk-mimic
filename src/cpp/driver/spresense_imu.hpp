// src/cpp/driver/spresense_imu.hpp
#pragma once
#include "../interface/imu_sensor.hpp"
#include "../lib/imu_mount.hpp"
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

    // IMU 取り付け回転を設定 (robot_config の imu_mount_rpy_deg)。
    // v_robot = Rz(yaw)·Ry(pitch)·Rx(roll) · v_imu。既定は恒等 (取り付け一致)。
    // accel / gyro / quaternion の全部に適用され、euler は回転後 quat から計算。
    // Open() 前に呼ぶこと (読み取りスレッドと排他していないため)。
    void SetMountRotation(double roll_deg, double pitch_deg, double yaw_deg);

private:
    void ReadThread();
    bool ValidateChecksum(const uint8_t* packet, size_t len);
    bool ParsePacket(const uint8_t* packet, size_t len);
    double GetNowSec();  // timestamp を秒で取得

    std::string port_;
    int baudrate_;
    int serial_fd_;

    // 取り付け回転 (SetMountRotation で設定、既定は恒等)
    imu_mount::Quat mount_quat_{1.0, 0.0, 0.0, 0.0};
    bool mount_identity_ = true;

    std::thread read_thread_;
    std::atomic<bool> running_;

    std::mutex data_mutex_;
    ImuData latest_data_;

    std::vector<uint8_t> buffer_;
};
