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

// 取り付け回転の数学は lib/imu_mount.hpp (header-only)。
// 旧実装にあった Euler レベルの Z+90° ハードコード補正は廃止 ——
// robot_config の imu_mount_rpy_deg + SetMountRotation で指定する
// (取り付けがロボット座標系と一致なら既定の恒等のまま)。

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
    // 注: Spresense は「シリアルを一度 close すると次の open で沈黙し、USB
    // 抜き差し (電源断) でしか復帰しない」挙動をする (実機確認済み。DTR/RTS
    // トグルでは復帰しない = HUPCL の問題ではない)。ノード再起動をまたいで
    // fd を掴み続けるため、必ず keep_serial_open.bash を先に起動しておくこと。

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

void SpresenseImu::SetMountRotation(double roll_deg, double pitch_deg,
                                    double yaw_deg)
{
    mount_quat_ = imu_mount::RpyDegToQuat(roll_deg, pitch_deg, yaw_deg);
    mount_identity_ =
        (roll_deg == 0.0 && pitch_deg == 0.0 && yaw_deg == 0.0);
    std::cout << "IMU mount rotation: rpy_deg = ["
              << roll_deg << ", " << pitch_deg << ", " << yaw_deg << "]"
              << (mount_identity_ ? " (identity)" : "") << std::endl;
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

    // 取り付け回転を accel / gyro / quaternion の全部に適用してから
    // ImuData に格納する (全フィールドがロボット座標系に揃う。
    // 旧実装は euler だけ補正して生ベクトルは IMU 座標系のままだった)。
    imu_mount::Quat q_imu{static_cast<double>(q0), static_cast<double>(q1),
                          static_cast<double>(q2), static_cast<double>(q3)};
    std::array<double, 3> acc{static_cast<double>(ax), static_cast<double>(ay),
                              static_cast<double>(az)};
    std::array<double, 3> gyr{static_cast<double>(gx), static_cast<double>(gy),
                              static_cast<double>(gz)};

    imu_mount::Quat q_robot = q_imu;
    if (!mount_identity_) {
        acc = imu_mount::RotateVec(mount_quat_, acc);
        gyr = imu_mount::RotateVec(mount_quat_, gyr);
        q_robot = imu_mount::Multiply(q_imu, imu_mount::Conjugate(mount_quat_));
    }

    data.timestamp = GetNowSec();
    data.ax = acc[0];
    data.ay = acc[1];
    data.az = acc[2];
    data.gx = gyr[0];
    data.gy = gyr[1];
    data.gz = gyr[2];
    data.q0 = q_robot.w;
    data.q1 = q_robot.x;
    data.q2 = q_robot.y;
    data.q3 = q_robot.z;

    // クォータニオン (回転適用後) → オイラー角
    imu_mount::QuatToEuler(q_robot, data.roll, data.pitch, data.yaw);

    // データ更新
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = data;
    }

    return true;
}
