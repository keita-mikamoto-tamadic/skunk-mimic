#pragma once
// AUTO-GENERATED from src/data_format/command_data.json by tools/gen_data_format.py
// DO NOT EDIT. 再生成: python3 tools/gen_data_format.py
#include <cstdint>
#include <cstddef>
#include "enum_def.hpp"

// dualsense_input → stabilizer: 走行指令。バランス制御の外側ループの目標速度に入る (PID 出力への直接加算ではない — 出力に足しても外側ループに打ち消される)。受け側は 300ms 途絶で 0 扱いにするので、送り側は ~50Hz で送り続けること
struct DriveCommand {
    double timestamp;  // 送信時刻 [s] (参考。staleness 判定は受信側の単調時計)
    double forward;  // 前後: 正規化操作量 -1..1 (前進が正)。物理速度へのスケールは制御側 (angle_pid の kMaxDriveWheelVel) が持つ — 送り側は物理単位を知らない
    double yaw;  // 旋回: 正規化操作量 -1..1 (右に倒すと正 = 右旋回のつもり。実機の回転方向は要確認)。左右ホイールへの差動 [rad/s] へのスケールは制御側 (kMaxYawWheelDiff) が持つ
    double height;  // 姿勢 (重心高さ) の変化レート: 正規化操作量 -1..1 (上げるが正)。制御側 (angle_pid の posture IK) が積分して目標の重心高さにし、倒立点を変えずに hip/knee を動かす。物理スケール [m/s] は制御側 (kMaxComHeightVel) が持つ
};
static_assert(sizeof(DriveCommand) == 32, "DriveCommand size mismatch vs command_data.json");
static_assert(offsetof(DriveCommand, timestamp) == 0, "DriveCommand.timestamp offset mismatch vs command_data.json");
static_assert(offsetof(DriveCommand, forward) == 8, "DriveCommand.forward offset mismatch vs command_data.json");
static_assert(offsetof(DriveCommand, yaw) == 16, "DriveCommand.yaw offset mismatch vs command_data.json");
static_assert(offsetof(DriveCommand, height) == 24, "DriveCommand.height offset mismatch vs command_data.json");

