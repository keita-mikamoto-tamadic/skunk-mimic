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

    // dummy はパラメータ読み出し非対応
    bool ReadParam(int, int, uint8_t*, int) override { return false; }
    bool ReadAllParams(int, uint8_t*, int) override { return false; }
    bool WriteParam(int, int, const uint8_t*, uint8_t*, uint8_t*, int) override { return false; }
    bool SaveAllParams(int, int) override { return false; }
    bool LoadDefaultParams(int, uint8_t*, int) override { return false; }
    bool Calibrate(int, float, float*, int) override { return false; }
    bool AnyValPosOffset(int, float, float*, int) override { return false; }

private:
    double dummy_pos_ = 0.0;
    double dummy_vel_ = 0.0;
};
