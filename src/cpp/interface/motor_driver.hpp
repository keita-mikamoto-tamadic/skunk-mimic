#pragma once
#include <vector>
#include <string>
#include <cstdint>
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

    // パラメータ読み出し(scalar, 単フレーム)。
    // out_value4 に 4byte の現在値を書き、成功なら true。
    // パラメータ機能を持たないドライバ(moteus/dummy)は false を返す。
    virtual bool ReadParam(int device_id, int param_index,
                           uint8_t* out_value4, int timeout_ms) = 0;

    // 全パラメータ読み出し(マルチフレーム)。26 scalar + LUT を ParamScalars(360byte)
    // として out_dump に書き出す。パラメータ機能を持たないドライバ(moteus/dummy)は false。
    virtual bool ReadAllParams(int device_id, uint8_t* out_dump,
                               int timeout_ms) = 0;

protected:
    MotorDriver() = default;
};
