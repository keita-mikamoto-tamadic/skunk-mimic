#pragma once
#include <vector>
#include <string>
#include "../lib/shm_data_format.hpp"
#include "../lib/robot_config.hpp"

class MotorDriver {
public:
    virtual ~MotorDriver() = default;

    MotorDriver(const MotorDriver&) = delete;
    MotorDriver& operator=(const MotorDriver&) = delete;

    virtual bool Open(const std::string& device) = 0;
    virtual void Close() = 0;

    virtual void SendCommands(const std::vector<AxisRef>& commands,
                              const std::vector<AxisConfig>& axes) = 0;
    virtual void SendQueries(const std::vector<AxisConfig>& axes) = 0;
    virtual std::vector<AxisAct> ReceiveStatus(
                              const std::vector<AxisConfig>& axes,
                              int timeout_ms) = 0;

    // 全軸 OFF 送信（シャットダウン時）
    virtual void SendAllOff(const std::vector<AxisConfig>& axes) = 0;

protected:
    MotorDriver() = default;
};
