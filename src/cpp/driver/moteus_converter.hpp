#pragma once
#include <cstdint>
#include <cstddef>
#include "../lib/enum_def.hpp"
#include "../lib/shm_data_format.hpp"

class MoteusConverter {
public:
    // コマンド → CAN フレーム構築（AxisRef.motor_state で分岐）
    // AxisRef は rad 単位のまま渡す。内部で rad→rev 変換する。
    // motdir: モーター方向 (1 or -1)
    size_t BuildCommandFrame(uint8_t* buf, const AxisRef& ref, int motdir);

    // エンコーダ位置リセットフレーム構築
    // position_rad: rad 単位。内部で rad→rev 変換する。
    size_t BuildResetPosition(uint8_t* buf, double position_rad, int motdir);

    // レスポンス解析 → AxisAct に rad 単位で書き込み（内部で rev→rad 変換）
    bool ParseResponse(const uint8_t* data, size_t len, AxisAct& act, int motdir);

    // CAN arbitration ID 生成
    uint32_t GetArbId(int device_id);
};