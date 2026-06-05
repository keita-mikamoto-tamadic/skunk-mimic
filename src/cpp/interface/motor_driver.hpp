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

    // 個別パラメータ設定(cmd=103)。value4 を書き、返信の old/new 値を out_old4/out_new4 へ。
    // パラメータ機能を持たないドライバ(moteus/dummy)は false。
    virtual bool WriteParam(int device_id, int param_index, const uint8_t* value4,
                            uint8_t* out_old4, uint8_t* out_new4, int timeout_ms) = 0;

    // 全パラメータセーブ(cmd=100, EEPROM 永続化)。完了(done=1)で true。
    // パラメータ機能を持たないドライバ(moteus/dummy)は false。
    virtual bool SaveAllParams(int device_id, int timeout_ms) = 0;

    // 全パラメータ初期値ロード(cmd=101)。返信(初期値)を ParamScalars(360byte)へ。
    // パラメータ機能を持たないドライバ(moteus/dummy)は false。
    virtual bool LoadDefaultParams(int device_id, uint8_t* out_dump,
                                   int timeout_ms) = 0;

    // 電気角キャリブ(cmd=1)。volt_d 印加でモータが回り、完了(done=1)まで数秒。
    // 完了時の機械角を out_pos へ書く。timeout_ms は長めに(数秒)。
    // パラメータ機能を持たないドライバ(moteus/dummy)は false。
    virtual bool Calibrate(int device_id, float volt_d,
                           float* out_pos, int timeout_ms) = 0;

    // 現在位置設定(cmd=110)。現在の機械角を target_pos として読ませる。
    // ファームが適用した offset を out_offset へ書く。
    // パラメータ機能を持たないドライバ(moteus/dummy)は false。
    virtual bool ZeroPosOffset(int device_id, float target_pos,
                               float* out_offset, int timeout_ms) = 0;

protected:
    MotorDriver() = default;
};
