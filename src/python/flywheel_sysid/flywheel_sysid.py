"""flywheel ホイール軸 SysID 励起+記録ノード (FOCTIVE, RCM バイパス)。

dataflow_flywheel_sysid.yaml の dynamic ノード。振子ボディをクランプ固定した状態で、
docker_ws/docker_isaaclab/src/flywheel_sim/scripts/sysid_excite.py と同一の位置励起
(±step + 対数チャープ、振幅 1/f で速度上限 --speed) を POSITION_PD (インピーダンス) で
再生し、motor_status イベント(= DCM 1ms tick)ごとに「記録→指令送信」して
シム側 sysid_record.py 互換の CSV (t,q1,dq1,cmd1 @1kHz) を書き出す。

起動 (dataflow 起動後、別ターミナル):
    cd src/python && ROBOT_CONFIG=robot_config/fly_wheel.json \\
        uv run flywheel_sysid/flywheel_sysid.py [--step 0.05 --amplitude 0.1 --f1 10]

出力 (既定 robot_file/fly_wheel/sysid_data/ = isaaclab コンテナの
/workspace/robot_file/fly_wheel/sysid_data/):
    record_<stamp>.csv       t,q1,dq1,cmd1 (1kHz, %.7g — sysid_record.py と同形式)
    excite_<stamp>.csv       t,q_cmd (--rate Hz, sysid_excite.py と同形式。シム側リプレイ用)
    record_<stamp>.meta.json 引数・ゲイン読み戻し・git rev など
中断 (Ctrl-C / dora stop / ガード発火) 時は OFF 送信後に record_<stamp>_aborted.csv を保存。

ゲイン: firmware には書き込まない。起動時に imp_kp/imp_kd を pread し、励起設計の前提
(--kp/--kd、シム PD と同値) と不一致なら中止する。設定自体は foctive_controller で
事前に行うこと (setting pwrite 27 <kp> / 28 <kd>)。指令側は kp_scale=kv_scale=1.0。
transport が "dummy" の場合 (ハード無し確認) は --skip-settings を付けること
(DummyDriver は settings 未実装)。dummy では位置/速度の窓ガードも自動で無効になる。
"""

from dora import Node
import pyarrow as pa
import argparse
import json
import math
import os
import signal
import struct
import subprocess
import sys
import time
from datetime import datetime

import numpy as np

# lib を import パスに追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.axis_data_format import (  # 自動生成(axis_data.json 正本)
    AxisRef, pack_axis_ref, unpack_axis_act, AXIS_ACT_SIZE,
    SettingsRequest, pack_settings_request, unpack_settings_result,
    PARAM_SCALARS_WIRE_INDEX,
)
from lib import robot_config

# <repo>/src/python/flywheel_sysid/flywheel_sysid.py → 3 階層上がリポジトリルート
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

# MotorState (enum_def.hpp と一致)
MOTOR_OFF = 0
MOTOR_POSITION_PD = 8  # インピーダンス制御: ref_val=pos, ref_val_1=vel, ref_val_2=FFトルク

# SettingsCmd (foctive_protocol.hpp と一致)
CMD_PREAD = 104
CMD_SETPOS = 110

# wire index 0-6 は uint32、それ以外は float ビット (foctive_controller.py と同じ規約)
INT_PARAM_INDICES = {0, 1, 2, 3, 4, 5, 6}

TICK_HZ = 1000.0          # DCM tick = dataflow yaml の dora/timer/millis/1
SETTINGS_TIMEOUT = 0.5    # settings_result 待ち [s]
EVENT_TIMEOUT = 1.0       # RUN 中にイベントが途絶えたら異常 [s]
PRE_ROLL_TICKS = 500      # 記録前に pos=0 を送って係合確認する時間 (0.5 s)
MIN_FW_VEL_LIMIT = 16.0   # firmware velocity_limit がこれ未満ならチャープが歪むので中止
VEL_ABORT = 20.0          # [rad/s] ガード (チャープ速度上限 15 + 余裕)
POS_ABORT = 1.0           # [rad] ガード (指令は ±0.5 以内)
SILENT_ABORT = 50         # silent_ticks (= 50 ms 無応答) で中止


class AbortRun(Exception):
    """OFF 送信 → 部分保存で終了すべき異常。"""


def build_excitation(args):
    """sysid_excite.py と同一の励起信号 (--rate Hz グリッド) を返す。"""
    def hold(value, seconds):
        return np.full(round(seconds * args.rate), value)

    # steps: 0 -> +h -> -h -> 0, 0.5 s each
    steps = np.concatenate([hold(0.0, 0.5), hold(args.step, 0.5),
                            hold(-args.step, 0.5), hold(0.0, 0.5)])

    t = np.arange(round(args.duration * args.rate)) / args.rate
    k = np.log(args.f1 / args.f0) / args.duration
    freq = args.f0 * np.exp(k * t)
    phase = 2 * np.pi * args.f0 * (np.exp(k * t) - 1) / k
    amplitude = np.minimum(args.amplitude, args.speed / (2 * np.pi * freq))
    chirp = amplitude * np.sin(phase)

    return np.concatenate([steps, chirp, hold(0.0, 1.0)])


def float_bits(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def interpret_param(index, value_u32):
    if index in INT_PARAM_INDICES:
        return value_u32
    return struct.unpack("<f", struct.pack("<I", value_u32))[0]


def send_ref(node, rec):
    node.send_output("motor_commands",
                     pa.array(list(pack_axis_ref(rec)), type=pa.uint8()))


def send_off(node):
    send_ref(node, AxisRef(motor_state=MOTOR_OFF))


def next_event(node, timeout):
    """node.next のラッパ: STOP は AbortRun に変換。"""
    ev = node.next(timeout=timeout)
    if ev is not None and ev["type"] == "STOP":
        raise AbortRun("dora STOP")
    return ev


def unpack_first_axis(ev):
    buf = bytes(ev["value"].to_pylist())
    if len(buf) < AXIS_ACT_SIZE:
        raise AbortRun(f"motor_status が短すぎる ({len(buf)} B)")
    return unpack_axis_act(buf)


def settings_round_trip(node, req, timeout=SETTINGS_TIMEOUT):
    """settings_request を送り、motor_status を読み捨てつつ settings_result を待つ。"""
    node.send_output("settings_request",
                     pa.array(list(pack_settings_request(req)), type=pa.uint8()))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ev = next_event(node, timeout=0.1)
        if ev is None or ev["type"] != "INPUT":
            continue
        if ev["id"] == "settings_result":
            return unpack_settings_result(bytes(ev["value"].to_pylist()))
        # motor_status 等は読み捨て
    return None


def do_settings(node, device_id, args):
    """ゲイン検証 (pread のみ、書き込まない) + setpos 0。読み戻し dict を返す。"""
    idx = PARAM_SCALARS_WIRE_INDEX
    readbacks = {}

    # 1) 同定・meta 用パラメータ読み出し (firmware には何も書かない)
    for name in ("imp_kp", "imp_kd", "torq_const", "gear_ratio", "rot_dir",
                 "velocity_limit", "trq_out_mx"):
        req = SettingsRequest(device_id=device_id, cmd=CMD_PREAD,
                              param_index=idx[name], value=0)
        res = settings_round_trip(node, req)
        if res is None or not res.ok:
            raise AbortRun(f"pread {name} 失敗 (応答なし/エラー)")
        readbacks[name] = interpret_param(res.param_index, res.value)
        print(f"[flywheel_sysid] {name} = {readbacks[name]:g}")

    # 2) ゲインが励起設計の前提 (シム PD) と一致しているか確認。不一致なら止める。
    #    設定はユーザーが foctive_controller で行う:
    #      setting pwrite 27 <kp> / setting pwrite 28 <kd> (+必要なら setting save)
    for name, expected in (("imp_kp", args.kp), ("imp_kd", args.kd)):
        if not math.isclose(readbacks[name], expected, rel_tol=1e-3):
            raise AbortRun(
                f"{name}={readbacks[name]:g} が期待値 {expected:g} と不一致 — "
                f"foctive_controller で `setting pwrite {idx[name]} {expected:g}` "
                f"してから再実行 (または --kp/--kd を実機側の値に合わせる)")

    # 3) firmware 側クランプの確認 (黙ってチャープが歪むのを防ぐ)
    if readbacks["velocity_limit"] < MIN_FW_VEL_LIMIT:
        raise AbortRun(
            f"firmware velocity_limit={readbacks['velocity_limit']:g} < "
            f"{MIN_FW_VEL_LIMIT} rad/s — チャープ(上限 {args.speed} rad/s)が歪む")

    # 4) クランプ静止位置を q=0 に定義 (励起は 0 始まり/0 終わりなので係合時に跳ばない)
    req = SettingsRequest(device_id=device_id, cmd=CMD_SETPOS,
                          param_index=0, value=float_bits(0.0))
    res = settings_round_trip(node, req)
    if res is None or not res.ok:
        raise AbortRun("setpos 0.0 失敗 (応答なし/エラー)")
    print("[flywheel_sysid] setpos: 現在位置を 0 に設定")

    return readbacks


def save_outputs(out_dir, stamp, aborted, n_done, q, dq, cmd_1k, q_cmd_grid,
                 args, meta_extra):
    os.makedirs(out_dir, exist_ok=True)
    suffix = "_aborted" if aborted else ""
    record_path = os.path.join(out_dir, f"record_{stamp}{suffix}.csv")
    excite_path = os.path.join(out_dir, f"excite_{stamp}.csv")
    meta_path = os.path.join(out_dir, f"record_{stamp}{suffix}.meta.json")

    n = n_done
    t = np.arange(n) / TICK_HZ
    # sysid_record.py:94 と同形式 (ヘッダ・%.7g)
    np.savetxt(record_path, np.column_stack([t, q[:n], dq[:n], cmd_1k[:n]]),
               delimiter=",", header="t,q1,dq1,cmd1", comments="", fmt="%.7g")
    # sysid_excite.py と同形式 (シム側で同一入力をリプレイするため)
    t_grid = np.arange(len(q_cmd_grid)) / args.rate
    np.savetxt(excite_path, np.column_stack([t_grid, q_cmd_grid]),
               delimiter=",", header="t,q_cmd", comments="", fmt="%.6g")

    try:
        git_rev = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT,
            capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        git_rev = ""
    meta = {"args": vars(args), "git_rev": git_rev, "tick_hz": TICK_HZ,
            "n_samples": n, "aborted": aborted, **meta_extra}
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print(f"[flywheel_sysid] wrote {record_path} ({n} rows)")
    print(f"[flywheel_sysid] wrote {excite_path} / {os.path.basename(meta_path)}")
    print(f"[flywheel_sysid] container path: "
          f"/workspace/robot_file/fly_wheel/sysid_data/{os.path.basename(record_path)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    # 励起パラメータ (sysid_excite.py と同じ既定値)
    parser.add_argument("--rate", type=float, default=100.0, help="指令グリッド [Hz]")
    parser.add_argument("--amplitude", type=float, default=0.5, help="最大位置振幅 [rad]")
    parser.add_argument("--speed", type=float, default=15.0, help="チャープ速度上限 [rad/s]")
    parser.add_argument("--step", type=float, default=0.1, help="ステップ高さ [rad]")
    parser.add_argument("--f0", type=float, default=0.5, help="チャープ開始周波数 [Hz]")
    parser.add_argument("--f1", type=float, default=30.0, help="チャープ終了周波数 [Hz]")
    parser.add_argument("--duration", type=float, default=20.0, help="チャープ長 [s]")
    # 実機側
    parser.add_argument("--kp", type=float, default=30.0,
                        help="imp_kp の期待値 [N·m/rad] (読み出して不一致なら中止)")
    parser.add_argument("--kd", type=float, default=0.2,
                        help="imp_kd の期待値 [N·m·s/rad] (同上)")
    parser.add_argument("--skip-settings", action="store_true",
                        help="settings フェーズを飛ばす (transport=dummy 用)")
    parser.add_argument("--out-dir",
                        default=os.path.join(REPO_ROOT, "robot_file", "fly_wheel", "sysid_data"))
    args = parser.parse_args()

    # config (dynamic ノードなのでシェルの ROBOT_CONFIG。相対パスはリポジトリルート基準)
    cfg_path = os.environ.get("ROBOT_CONFIG", "robot_config/fly_wheel.json")
    if not os.path.isabs(cfg_path) and not os.path.exists(cfg_path):
        cfg_path = os.path.join(REPO_ROOT, cfg_path)
    cfg = robot_config.load_from_file(cfg_path)
    axis = cfg.axes[0]
    is_dummy = cfg.transport == "dummy"
    print(f"[flywheel_sysid] config: {cfg.robot_name} axis='{axis.name}' "
          f"device_id={axis.device_id} transport={cfg.transport}")
    if is_dummy:
        print("[flywheel_sysid] dummy transport: 位置/速度の窓ガードは無効")

    # 励起信号: --rate グリッド生成 → 1kHz へ ZOH (シムの sysid_record.py:86 と同一規約)
    q_cmd_grid = build_excitation(args)
    ratio = round(TICK_HZ / args.rate)
    cmd_1k = np.repeat(q_cmd_grid, ratio)
    n_total = len(cmd_1k)
    print(f"[flywheel_sysid] excitation: {len(q_cmd_grid)} samples @{args.rate:g} Hz "
          f"→ {n_total} ticks ({n_total / TICK_HZ:.1f} s)")

    q = np.empty(n_total)
    dq = np.empty(n_total)
    k = 0                   # 記録済みサンプル数
    meta_extra = {"readbacks": {}, "wall_run_s": None,
                  "started": datetime.now().isoformat(timespec="seconds"),
                  "abort_reason": None,
                  "note": "row k の指令が firmware に届くのは ~1 tick 後 "
                          "(シムの record-then-apply と同等の ≤1 sample skew)"}
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    stop_requested = False

    def on_signal(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    node = Node("flywheel_sysid")
    aborted = True  # 正常完了時のみ False にする
    try:
        # --- WAIT_DCM: 最初の motor_status を待つ (サーボは OFF のまま) ---
        print("[flywheel_sysid] waiting for motor_status ...")
        while True:
            if stop_requested:
                raise AbortRun("signal")
            ev = next_event(node, timeout=1.0)
            if ev is not None and ev["type"] == "INPUT" and ev["id"] == "motor_status":
                act = unpack_first_axis(ev)
                if act.fault != 0:
                    raise AbortRun(f"初期状態で fault={act.fault}")
                if act.silent_ticks != 0:
                    raise AbortRun(f"モータ無応答 (silent_ticks={act.silent_ticks})")
                break

        # --- SETTINGS: ゲイン書き込み・読み戻し・setpos 0 (サーボ OFF 前提) ---
        if args.skip_settings:
            print("[flywheel_sysid] --skip-settings: ゲイン書き込みをスキップ")
        else:
            meta_extra["readbacks"] = do_settings(node, axis.device_id, args)

        # --- PRE-ROLL: pos=0 で係合し追従を確認 (記録外) ---
        print(f"[flywheel_sysid] pre-roll {PRE_ROLL_TICKS / TICK_HZ:.1f} s (pos=0)")
        pre = 0
        while pre < PRE_ROLL_TICKS:
            if stop_requested:
                raise AbortRun("signal")
            ev = next_event(node, timeout=EVENT_TIMEOUT)
            if ev is None:
                raise AbortRun("pre-roll 中に motor_status が途絶")
            if ev["type"] != "INPUT" or ev["id"] != "motor_status":
                continue
            act = unpack_first_axis(ev)
            if act.fault != 0:
                raise AbortRun(f"pre-roll 中に fault={act.fault}")
            send_ref(node, AxisRef(motor_state=MOTOR_POSITION_PD, ref_val=0.0,
                                   kp_scale=1.0, kv_scale=1.0))
            pre += 1
        if not is_dummy and abs(act.position) > 0.1:
            raise AbortRun(f"pre-roll 後も位置が 0 に収束しない ({act.position:+.3f} rad)")

        # --- RUN: ガード → 記録 → 指令送信 (record-then-apply) ---
        print("[flywheel_sysid] RUN")
        t_run = time.monotonic()
        while k < n_total:
            if stop_requested:
                raise AbortRun("signal")
            ev = next_event(node, timeout=EVENT_TIMEOUT)
            if ev is None:
                raise AbortRun(f"RUN 中に motor_status が途絶 (k={k})")
            if ev["type"] != "INPUT" or ev["id"] != "motor_status":
                continue
            act = unpack_first_axis(ev)
            if act.fault != 0:
                raise AbortRun(f"fault={act.fault} (k={k})")
            if act.silent_ticks > SILENT_ABORT:
                raise AbortRun(f"無応答 silent_ticks={act.silent_ticks} (k={k})")
            if not is_dummy and abs(act.velocity) > VEL_ABORT:
                raise AbortRun(f"|velocity|={abs(act.velocity):.1f} > {VEL_ABORT} (k={k})")
            if not is_dummy and abs(act.position) > POS_ABORT:
                raise AbortRun(f"|position|={abs(act.position):.2f} > {POS_ABORT} (k={k})")
            q[k] = act.position
            dq[k] = act.velocity
            send_ref(node, AxisRef(motor_state=MOTOR_POSITION_PD, ref_val=cmd_1k[k],
                                   kp_scale=1.0, kv_scale=1.0))
            k += 1
            if k % 5000 == 0:
                print(f"[flywheel_sysid] {k}/{n_total} ({k / TICK_HZ:.0f} s)")

        wall = time.monotonic() - t_run
        meta_extra["wall_run_s"] = wall
        expected = n_total / TICK_HZ
        if wall - expected > 0.05:
            print(f"[flywheel_sysid] 警告: RUN 実時間 {wall:.3f} s > 期待 {expected:.3f} s "
                  f"— イベント欠落の疑い (yaml の queue_size を増やす / ノード負荷を見直す)")
            meta_extra["note"] += f"; WALL-TIME MISMATCH {wall:.3f}s vs {expected:.3f}s"
        aborted = False

    except AbortRun as e:
        meta_extra["abort_reason"] = str(e)
        print(f"[flywheel_sysid] ABORT: {e}")
    finally:
        # OFF を先に送る (DCM の foctive 経路は最終フレームを毎 tick 再送するので OFF が維持される)
        try:
            send_off(node)
            print("[flywheel_sysid] sent MOTOR_OFF")
        except Exception as e:
            print(f"[flywheel_sysid] OFF 送信失敗: {e}", file=sys.stderr)
        if k > 0 or not aborted:
            save_outputs(args.out_dir, stamp, aborted, k, q, dq, cmd_1k,
                         q_cmd_grid, args, meta_extra)
        else:
            print("[flywheel_sysid] 記録 0 行のため CSV は保存しない")

    sys.exit(1 if aborted else 0)


if __name__ == "__main__":
    main()
