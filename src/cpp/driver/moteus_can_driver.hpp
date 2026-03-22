#pragma once
#include "../interface/motor_driver.hpp"
#include "socket_can_comm.hpp"
#include "moteus_converter.hpp"
#include <set>

class MoteusCanDriver : public MotorDriver {
public:
    MoteusCanDriver() = default;
    ~MoteusCanDriver() override { Close(); }

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
    SocketCanComm comm_;
    MoteusConverter converter_;
    std::set<int> expected_ids_;
};
