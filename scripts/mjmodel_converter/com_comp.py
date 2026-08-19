#!/usr/bin/env python3
"""
com_comp — 実測の倒立 pitch に合わせて model.json の重心補正 (com_offset) を決める。

CAD (Fusion CSV) の質量特性で作った MuJoCo モデルは、配線・バッテリー・搭載物など CAD に無い
質量のぶん倒立点 (standing 姿勢で全体重心が車軸の真上に来る base pitch) が実機とずれる。
実機の倒立点は初期姿勢で平衡する角度を実測したもの (angle_pid の kTargetPitch) で確かなので、
それを正とし、モデル側の 1 body (既定 base_link) の重心を body 座標 x 方向にだけ動かして
倒立点を一致させる。

1 姿勢の pitch 1 個からは「その姿勢での全体重心の車軸まわり角度」というスカラー 1 個しか
同定できないので、補正は 1 body・1 方向に限る (複数に分配する根拠が無い)。IMU 取り付け角や
関節原点のズレもこの補正に吸収されることに注意 (standing 姿勢では一致するが、他の姿勢では
一致を保証しない)。

やること:
  1. model.json を読み、対象 body の既存 com_offset を 0 にした (= CAD そのままの) モデルを組む
  2. standing 姿勢 (robot_config の initial_position) で倒立 pitch を求根 → CAD の倒立点
  3. 対象 body の重心 x を動かしたときの倒立 pitch が目標値になる dx を求根
  4. model.json の対象 body に "com_offset": [dx, 0, 0] と根拠コメント "_com_offset" を書く
     (他の行・整形は触らない)
  5. converter を呼んで sim/<model>.xml を再生成 (--no-regen で省略)

使い方:
  cd scripts
  uv run mjmodel_converter/com_comp.py 0.165                 # 実測倒立 pitch [rad] (stabilizer が見る ImuData.pitch の符号)
  uv run mjmodel_converter/com_comp.py 0.165 --dry-run       # 計算だけ (書き込み・再生成なし)
  uv run mjmodel_converter/com_comp.py 0.165 --param-dir sim/fusion_param/<robot> --body base_link
  uv run mjmodel_converter/com_comp.py --reset               # com_offset を外して CAD の値に戻す

pitch の符号は mujoco_backend.quat_to_euler (= stabilizer の ImuData.pitch) と同じ: base を +y 軸
まわりに回した角。angle_pid の kTargetPitch と同じ量なのでそのまま渡せる。
"""

from __future__ import annotations

import argparse
import datetime as _dt
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mjmodel_converter as conv  # noqa: E402

REPO_ROOT = conv.REPO_ROOT


def build_model(spec: dict, param_dir: Path, rc: dict | None, meshdir_abs: Path):
    """spec から MjModel と builder を組む (keyframe 無しで十分。倒立点計算は姿勢を自前で入れる)"""
    builder = conv.MjcfBuilder(spec, param_dir, rc)
    builder.build()
    builder.meshdir_abs = meshdir_abs
    return builder.load_mjmodel(), builder


def solve_offset(spec: dict, param_dir: Path, rc: dict | None, meshdir_abs: Path,
                 body_name: str, target: float, axis: int = 0) -> tuple[float, float, float, float]:
    """(CAD の倒立 pitch, 必要な dx, 補正後の倒立 pitch, 車軸上の重心高さ) を返す"""
    import mujoco

    # 対象 body の com_offset を外した状態 (CAD そのまま) から始める
    for b in spec["bodies"]:
        if b["name"] == body_name:
            b.pop("com_offset", None)
            break
    else:
        sys.exit(f"error: body '{body_name}' not in spec")

    model, builder = build_model(spec, param_dir, rc, meshdir_abs)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if bid < 0:
        sys.exit(f"error: body '{body_name}' not found in the built model")

    base_ipos = model.body_ipos[bid].copy()
    pitch_cad = conv.balance_pitch(model, builder)
    _, h = conv.com_over_axle(model, mujoco.MjData(model), builder, pitch_cad)

    def pitch_for(dx: float) -> float:
        # body_ipos は body 座標系の重心位置。mj_forward の subtree_com はこれを使うので、
        # XML を作り直さずに MjModel 上で動かして求根できる
        model.body_ipos[bid] = base_ipos
        model.body_ipos[bid][axis] += dx
        return conv.balance_pitch(model, builder)

    # dx の探索区間: 全体重心の必要移動量 ≈ h·tan(Δpitch) を body 質量比で拡大したものを中心に ±3 倍
    m_total = float(model.body_subtreemass[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, builder.bodies[0]["name"])])
    m_body = float(model.body_mass[bid])
    guess = h * math.tan(target - pitch_cad) * m_total / m_body
    span = max(3.0 * abs(guess), 0.005)
    lo, hi = guess - span, guess + span
    f_lo, f_hi = pitch_for(lo) - target, pitch_for(hi) - target
    if f_lo * f_hi > 0:
        sys.exit(f"error: cannot bracket dx in [{lo:.4f}, {hi:.4f}] m (target {target} rad; CAD {pitch_cad:.4f} rad)")
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        f_mid = pitch_for(mid) - target
        if f_lo * f_mid <= 0:
            hi, f_hi = mid, f_mid
        else:
            lo, f_lo = mid, f_mid
    dx = 0.5 * (lo + hi)
    pitch_after = pitch_for(dx)
    model.body_ipos[bid] = base_ipos
    return pitch_cad, dx, pitch_after, h


# ---------------------------------------------------------------------------
# model.json の局所編集 (整形を保ったまま対象 body に com_offset を書く / 消す)
# ---------------------------------------------------------------------------

def _find_body_block(text: str, body_name: str) -> tuple[int, int]:
    """'"name": "<body>"' を含む最も内側の {...} の範囲 (start, end) を返す (end は '}' の次)"""
    m = re.search(r'"name"\s*:\s*"%s"' % re.escape(body_name), text)
    if not m:
        sys.exit(f"error: body '{body_name}' not found in model.json text")
    # 後ろ向きに対応する '{' を探す
    depth = 0
    i = m.start()
    while i >= 0:
        c = text[i]
        if c == "}":
            depth += 1
        elif c == "{":
            if depth == 0:
                break
            depth -= 1
        i -= 1
    start = i
    depth = 0
    j = start
    while j < len(text):
        c = text[j]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return start, j + 1
        j += 1
    sys.exit("error: unbalanced braces in model.json")


def write_offset(spec_path: Path, body_name: str, dx: float | None, note: str | None):
    text = spec_path.read_text(encoding="utf-8")
    start, end = _find_body_block(text, body_name)
    block = text[start:end]

    # 既存の com_offset / _com_offset 行を除去
    block = re.sub(r'\n[ \t]*"_com_offset"\s*:\s*"[^"\n]*",?', "", block)
    block = re.sub(r'\n[ \t]*"com_offset"\s*:\s*\[[^\]\n]*\],?', "", block)

    if dx is not None:
        # "part" 行の直後に挿入 (無ければ "name" 行の直後)。インデントはその行に合わせる
        anchor = re.search(r'\n([ \t]*)"part"\s*:[^\n]*', block) or re.search(r'\n([ \t]*)"name"\s*:[^\n]*', block)
        indent = anchor.group(1)
        insert = (f'\n{indent}"_com_offset": "{note}",'
                  f'\n{indent}"com_offset": [{dx:.6f}, 0.0, 0.0],')
        block = block[:anchor.end()] + insert + block[anchor.end():]
    # 末尾のカンマ整合: ブロック最後のプロパティの後ろに ',' が残らないように
    block = re.sub(r",(\s*\})$", r"\1", block)
    # 削除で「最後のプロパティにカンマ無し → 次の行がある」状態になることは無い (末尾のみ処理)
    text = text[:start] + block + text[end:]
    spec_path.write_text(text, encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target_pitch", nargs="?", type=float,
                    help="実測の倒立 pitch [rad] (angle_pid の kTargetPitch と同じ符号・同じ量)")
    ap.add_argument("--param-dir", default=str(conv.DEFAULT_PARAM_DIR),
                    help=f"model.json のディレクトリ (default: {conv.DEFAULT_PARAM_DIR.relative_to(REPO_ROOT)})")
    ap.add_argument("--body", default="base_link", help="重心を動かす body (default: base_link)")
    ap.add_argument("--dry-run", action="store_true", help="計算だけして model.json を書かない")
    ap.add_argument("--reset", action="store_true", help="com_offset を外して CAD の値に戻す (target_pitch 不要)")
    ap.add_argument("--no-regen", action="store_true", help="model.json 更新後の XML 再生成をしない")
    args = ap.parse_args(argv)

    param_dir = Path(args.param_dir)
    if not param_dir.is_absolute():
        param_dir = (Path.cwd() / param_dir).resolve() if (Path.cwd() / param_dir).exists() else (REPO_ROOT / args.param_dir).resolve()
    spec_path = param_dir / conv.SPEC_FILENAME
    spec = conv.load_spec(param_dir)
    rc = conv.load_robot_config(spec)
    meshdir_abs = (REPO_ROOT / "sim" / spec.get("meshdir", "meshes/")).resolve()

    if args.reset:
        write_offset(spec_path, args.body, None, None)
        print(f"removed com_offset from {args.body} in {spec_path.relative_to(REPO_ROOT)}")
    else:
        if args.target_pitch is None:
            ap.error("target_pitch is required (or use --reset)")
        target = args.target_pitch
        pitch_cad, dx, pitch_after, h = solve_offset(spec, param_dir, rc, meshdir_abs, args.body, target)
        print(f"standing pose, body={args.body}")
        print(f"  CAD balance pitch     : {pitch_cad:+.4f} rad ({math.degrees(pitch_cad):+.2f} deg)")
        print(f"  measured (target)     : {target:+.4f} rad ({math.degrees(target):+.2f} deg)  gap {math.degrees(target - pitch_cad):+.2f} deg")
        print(f"  CoM height above axle : {h:.4f} m")
        print(f"  => com_offset x       : {dx * 1000:+.2f} mm on {args.body}  (balance pitch after: {pitch_after:+.4f} rad)")
        if args.dry_run:
            print("dry-run: model.json not modified")
            return 0
        note = (f"com_comp.py {_dt.date.today().isoformat()}: standing 姿勢の倒立 pitch を実測 {target:.4f} rad に合わせる "
                f"(CAD {pitch_cad:.4f} rad, 差 {math.degrees(target - pitch_cad):+.2f} deg)。x のみ、この body のみ。"
                f"再計算は `uv run mjmodel_converter/com_comp.py {target}`")
        write_offset(spec_path, args.body, dx, note)
        print(f"wrote com_offset to {spec_path.relative_to(REPO_ROOT)}")

    if args.no_regen:
        return 0
    print("\nregenerating MJCF ...")
    return conv.main([str(param_dir)])


if __name__ == "__main__":
    sys.exit(main())
