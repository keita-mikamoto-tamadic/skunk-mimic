#pragma once

#include <cstdint>
#include <cstring>  // memcpy
#include "axis_data_format.hpp"  // param_scalars:: (src/data_format/axis_data.json から生成)

// FOCTIVE パラメータ定義。正本は src/data_format/axis_data.json の ParamScalars
// (index = FOCTIVE の param_index、repr = float/uint32 の解釈)。ここは生成物を
// Foctive の名前で使えるようにするだけ。ファームのパラメータが増減したら JSON を直して
// python3 tools/gen_data_format.py で再生成する (このファイルは触らなくてよい)。
namespace Foctive {

  // ファームの MotParam と同一レイアウト (index 順。LUT は index 8 の位置)。
  // cmd=101/102 はこの構造体をそのままマルチフレームで送ってくる。
  using MotParam = param_scalars::Wire;

  // 全パラメータ個数 (scalar + LUT)。cmd=100/102 の param_num に使う。
  // ファームの ParamIndex::kTotal と一致しないと cmd=255 (param_num 不一致) で弾かれる。
  constexpr uint8_t kParamNum = param_scalars::kCount;

  using ParamIndex = param_scalars::Index;
  using enum param_scalars::Index;  // kMotorId, kElecAngleOfs, kInvalid, ...

  using ParamDesc = param_scalars::Desc;
  inline constexpr const ParamDesc (&kParamDesc)[kParamNum] = param_scalars::kDesc;

  // 受信した生バイト列を MotParam の該当フィールドへ書き込む(cmd=104/102 のデコード用)
  inline void WriteParam(MotParam& p, ParamIndex idx, const uint8_t* src) {
    if (idx >= kParamNum) return;  // ファームとの定義ずれ (未知 index) は捨てる
    const ParamDesc& d = kParamDesc[idx];
    std::memcpy(reinterpret_cast<uint8_t*>(&p) + d.offset, src, d.size);
  }

  // MotParam の該当フィールド先頭ポインタ(送信・表示用)
  inline const uint8_t* ParamPtr(const MotParam& p, ParamIndex idx) {
    return reinterpret_cast<const uint8_t*>(&p) + kParamDesc[idx].offset;
  }
}  // namespace Foctive
