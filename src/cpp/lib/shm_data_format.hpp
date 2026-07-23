#pragma once
#include <cstdint>
#include "enum_def.hpp"

// dora で交換する構造体はすべて src/data_format/*.json を正本に自動生成する
// (tools/gen_data_format.py / CMake configure 時に再生成)。
// このヘッダは「まとめて include する」アグリゲータで、構造体定義は持たない。
//
//   axis_data.json   → AxisRef / AxisAct / SettingsRequest / SettingsResult / ParamScalars
//   sensor_data.json → ImuData / LatencyData / EstimatedState
#include "axis_data_format.hpp"
#include "sensor_data_format.hpp"
