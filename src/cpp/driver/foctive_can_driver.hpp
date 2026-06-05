#pragma once
#include "../lib/enum_def.hpp"
#include "../lib/foctive_protocol.hpp"
#include "../interface/motor_driver.hpp"
#include "socket_can_comm.hpp"
#include <set>
#include <vector>
#include <map>

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

  // 個別パラメータ読み出し(cmd=104, 単フレーム)。基底I/F経由で DCM から呼ぶ。
  // 送信→返信待ち→保持 MotParam に反映し、out_value4 に 4byte 値を書く。
  bool ReadParam(int device_id, int param_index,
                 uint8_t* out_value4, int timeout_ms) override;
  // 全パラメータ読み出し(cmd=102, マルチフレーム)。連結 → 保持 MotParam に反映し、
  // 26 scalar の生値(ParamScalars, 104byte)を out_scalars に書き出す。
  bool ReadAllParams(int device_id, uint8_t* out_dump, int timeout_ms) override;
  // 個別パラメータ設定(cmd=103)。書込→返信(old/new)→MotParam 更新 + old/new を返す。
  bool WriteParam(int device_id, int param_index, const uint8_t* value4,
                  uint8_t* out_old4, uint8_t* out_new4, int timeout_ms) override;
  // 全パラメータセーブ(cmd=100, EEPROM)。完了(done=1)で true。
  bool SaveAllParams(int device_id, int timeout_ms) override;
  // 保持中の MotParam を参照(FOCTIVE 専用, 確認・表示用)
  const Foctive::MotParam& Params(int device_id);

private:
  SocketCanComm comm_;
  std::set<int> expected_ids_;
  // SendQueries 用: 各軸の最後に送信したフレームを保持し再送する
  // (FOCTIVE には純粋 Query フレームが無く、状態を変えずに返信を得るため last を再送)
  std::vector<Foctive::CanFdFrame> last_frames_;
  // 設定モードで読み出したパラメータを device_id ごとに保持(確認用)
  std::map<int, Foctive::MotParam> params_;
};