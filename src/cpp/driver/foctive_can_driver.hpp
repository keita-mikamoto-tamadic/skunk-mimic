#pragma once
#include "../lib/enum_def.hpp"
#include "../lib/foctive_protocol.hpp"
#include "../interface/motor_driver.hpp"
#include "socket_can_comm.hpp"
#include <set>
#include <vector>

class FoctiveCanDriver : public MotorDriver {
public:
  FoctiveCanDriver() = default;
  ~FoctiveCanDriver() override {Close();}

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
  std::set<int> expected_ids_;
  // SendQueries 用: 各軸の最後に送信したフレームを保持し再送する
  // (FOCTIVE には純粋 Query フレームが無く、状態を変えずに返信を得るため last を再送)
  std::vector<Foctive::CanFdFrame> last_frames_;
};