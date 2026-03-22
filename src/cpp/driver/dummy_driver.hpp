#pragma once
#include "../interface/motor_driver.hpp"

class DummyDriver : public MotorDriver {
public:
    DummyDriver() = default;
    ~DummyDriver() override = default;

    bool Open(const std::string& device) override;
    void Close() override;

    void SendCommands(const std::vector<AxisRef>& commands,
                      const std::vector<AxisConfig>& axes) override;
    void SendQueries(const std::vector<AxisConfig>& axes) override;
    std::vector<AxisAct> ReceiveStatus(
                      const std::vector<AxisConfig>& axes,
                      int timeout_ms) override;
    void SendAllOff(const std::vector<AxisConfig>& axes) override;

private:
    double dummy_pos_ = 0.0;
    double dummy_vel_ = 0.0;
};
