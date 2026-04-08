// src/cpp/driver/spresense_imu.cpp
#include "spresense_imu.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <chrono>
#include <iostream>
#include <cmath>

namespace {
// クォータニオン → オイラー角変換 (Roll, Pitch, Yaw)
// w=q0, x=q1, y=q2, z=q3 (Madgwickフィルタ出力順)
// IMU取り付け補正: Z軸周りに+90°回転
void QuaternionToEuler(double w, double x, double y, double z,
                       double& roll, double& pitch, double& yaw)
{
    // まずIMU座標系でのオイラー角を計算
    // Roll (X軸回転)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double imu_roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y軸回転)
    double sinp = 2.0 * (w * y - z * x);
    double imu_pitch;
    if (std::abs(sinp) >= 1.0)
        imu_pitch = std::copysign(M_PI / 2.0, sinp);  // ジンバルロック時
    else
        imu_pitch = std::asin(sinp);

    // Yaw (Z軸回転)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double imu_yaw = std::atan2(siny_cosp, cosy_cosp);

    // Z軸周りに+90°回転の座標変換
    // IMUのX軸 → ロボットのY軸、IMUのY軸 → ロボットの-X軸
    roll  = imu_pitch;
    pitch = -imu_roll;
    yaw   = imu_yaw;
}
} // anonymous namespace

SpresenseImu::SpresenseImu(const std::string& port, int baudrate)
    : port_(port),
      baudrate_(baudrate),
      serial_fd_(-1),
      running_(false)
{
    buffer_.reserve(kPacketSize * 2);
    latest_data_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

SpresenseImu::~SpresenseImu()
{
    Close();
}

bool SpresenseImu::Open(const std::string& device)
{
    if (serial_fd_ >= 0) {
        return true;  // 既に開いている
    }

    serial_fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        std::cerr << "Failed to open " << device << ": " << strerror(errno) << std::endl;
        return false;
    }

    // シリアルポート設定
    struct termios options;
    tcgetattr(serial_fd_, &options);

    // ボーレート設定
    speed_t baud;
    switch (baudrate_) {
        case 115200:
            baud = B115200;
            break;
        case 921600:
            baud = B921600;
            break;
        default:
            baud = B921600;
            break;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 8N1設定
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;

    // Raw入力
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

    // タイムアウト設定
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    tcsetattr(serial_fd_, TCSANOW, &options);
    tcflush(serial_fd_, TCIOFLUSH);

    // DTR/RTSを設定
    int status;
    ioctl(serial_fd_, TIOCMGET, &status);
    status |= TIOCM_DTR | TIOCM_RTS;
    ioctl(serial_fd_, TIOCMSET, &status);

    std::cout << "Opened " << device << " at " << baudrate_ << " bps" << std::endl;

    // 読み取りスレッド開始
    running_ = true;
    read_thread_ = std::thread(&SpresenseImu::ReadThread, this);

    return true;
}

void SpresenseImu::Close()
{
    running_ = false;

    if (read_thread_.joinable()) {
        read_thread_.join();
    }

    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
        std::cout << "Closed serial port" << std::endl;
    }
}

ImuData SpresenseImu::GetLatestData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
}

bool SpresenseImu::Read(uint8_t* data, size_t* len, int timeout_ms)
{
    // IMU では GetLatestData() を使用するため、この関数は未実装
    (void)data;
    (void)len;
    (void)timeout_ms;
    return false;
}

double SpresenseImu::GetNowSec()
{
    using namespace std::chrono;
    auto now = system_clock::now().time_since_epoch();
    return duration_cast<duration<double>>(now).count();
}

void SpresenseImu::ReadThread()
{
    uint8_t read_buffer[256];

    while (running_) {
        int bytes_read = ::read(serial_fd_, read_buffer, sizeof(read_buffer));

        if (bytes_read > 0) {
            // バッファに追加
            buffer_.insert(buffer_.end(), read_buffer, read_buffer + bytes_read);

            // パケット探索
            while (buffer_.size() >= kPacketSize) {
                // ヘッダー探索
                auto it = buffer_.begin();
                bool header_found = false;

                for (; it < buffer_.end() - 1; ++it) {
                    if (*it == kPacketHeader1 && *(it + 1) == kPacketHeader2) {
                        header_found = true;
                        break;
                    }
                }

                if (!header_found) {
                    buffer_.clear();
                    break;
                }

                // ヘッダーより前のデータを削除
                if (it != buffer_.begin()) {
                    buffer_.erase(buffer_.begin(), it);
                }

                // 完全なパケットがあるか確認
                if (buffer_.size() >= kPacketSize) {
                    if (ValidateChecksum(buffer_.data(), kPacketSize)) {
                        ParsePacket(buffer_.data(), kPacketSize);
                    }
                    buffer_.erase(buffer_.begin(), buffer_.begin() + kPacketSize);
                } else {
                    break;
                }
            }

            // バッファサイズ制限
            if (buffer_.size() > kPacketSize * 10) {
                buffer_.clear();
            }
        } else if (bytes_read < 0 && errno != EAGAIN) {
            std::cerr << "Serial read error: " << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

bool SpresenseImu::ValidateChecksum(const uint8_t* packet, size_t len)
{
    if (len != kPacketSize) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < kPacketSize - 1; i++) {
        checksum ^= packet[i];
    }

    return checksum == packet[kPacketSize - 1];
}

bool SpresenseImu::ParsePacket(const uint8_t* packet, size_t len)
{
    if (len < kPacketSize) {
        return false;
    }

    if (packet[0] != kPacketHeader1 || packet[1] != kPacketHeader2) {
        return false;
    }

    ImuData data;
    float q0, q1, q2, q3;
    float gx, gy, gz;
    float ax, ay, az;

    // パケットから float 値を取得（参考コードのフォーマット）
    std::memcpy(&q0, &packet[2], sizeof(float));
    std::memcpy(&q1, &packet[6], sizeof(float));
    std::memcpy(&q2, &packet[10], sizeof(float));
    std::memcpy(&q3, &packet[14], sizeof(float));
    std::memcpy(&gx, &packet[18], sizeof(float));
    std::memcpy(&gy, &packet[22], sizeof(float));
    std::memcpy(&gz, &packet[26], sizeof(float));
    std::memcpy(&ax, &packet[30], sizeof(float));
    std::memcpy(&ay, &packet[34], sizeof(float));
    std::memcpy(&az, &packet[38], sizeof(float));

    // shm_data_format.hpp の ImuData にマッピング
    // Z軸周りに-90°回転（QuaternionToEulerと同じ座標変換）
    // IMU X → Robot -Y, IMU Y → Robot X, IMU Z → Robot Z
    data.timestamp = GetNowSec();
    data.ax = static_cast<double>(ay);
    data.ay = static_cast<double>(-ax);
    data.az = static_cast<double>(az);
    data.gx = static_cast<double>(gy);
    data.gy = static_cast<double>(-gx);
    data.gz = static_cast<double>(gz);
    data.q0 = static_cast<double>(q0);  // w
    data.q1 = static_cast<double>(q1);  // x
    data.q2 = static_cast<double>(q2);  // y
    data.q3 = static_cast<double>(q3);  // z

    // クォータニオン → オイラー角変換（座標系変換込み）
    QuaternionToEuler(data.q0, data.q1, data.q2, data.q3,
                      data.roll, data.pitch, data.yaw);

    // データ更新
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = data;
    }

    return true;
}
