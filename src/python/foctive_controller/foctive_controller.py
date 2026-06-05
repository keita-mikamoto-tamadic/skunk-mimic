from dora import Node
import pyarrow as pa
import sys
import os
import struct

# lib を import パスに追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.data_format import (  # 自動生成(axis_data.json 正本)
    AxisRef, pack_axis_ref,
    SettingsRequest, pack_settings_request, unpack_settings_result,
    unpack_param_scalars,
)

# ParamScalars のうち uint32 として表示するフィールド(残りは float ビットを再解釈)
INT_PARAM_FIELDS = {
    "motor_id", "device_id", "rot_dir", "mech_angle_dir",
    "elec_angle_dir", "mot_pole_pairs", "gear_enable",
}

# setting サブコマンド → SettingsCmd 番号 (foctive_protocol.hpp の SettingsCmd と一致)
SETTINGS_CMD = {
    "pread": 104,    # 個別パラメータ読み出し
    "pwrite": 103,   # 個別パラメータ設定 (未配線)
    "readall": 102,  # 全パラメータ読み出し (未配線, マルチフレーム)
    "save": 100,     # 全パラメータセーブ (未配線)
    "load": 101,     # 全パラメータ初期値で設定 (未配線)
    "calib": 1,      # 電気角キャリブ (未配線)
}

# uint32 として解釈するパラメータ index (それ以外は float)。index 8 は LUT(scalar外)
INT_PARAM_INDICES = {0, 1, 2, 3, 4, 5, 6}


def interpret_param(index, value_u32):
    if index in INT_PARAM_INDICES:
        return value_u32
    # float: 生ビットを float として再解釈
    return struct.unpack("<f", struct.pack("<I", value_u32))[0]

# MotorState (enum_def.hpp と一致)
MOTOR_OFF = 0
MOTOR_VOLTAGE = 7


def axis_ref_bytes(rec):
    # AxisRef(namedtuple) → uint8 配列 (dora 送信用)
    return pa.array(list(pack_axis_ref(rec)), type=pa.uint8())


node = Node("foctive_controller")

DEVICE_ID = 1  # 単軸テスト用

print("[foctive_controller] Commands:")
print("  v <volt_d> <volt_q> <vir_ang_freq> : 電圧制御 (例: v 0 1.0 20)")
print("  setting pread <param_index>        : パラメータ読み出し (cmd=104, サーボOFFで)")
print("  f : SERVO OFF")
print("  q : QUIT")

while True:
    try:
        line = input("> ").strip()
    except EOFError:
        break
    if not line:
        continue

    parts = line.split()
    cmd = parts[0].lower()

    if cmd == "q":
        node.send_output("motor_commands", axis_ref_bytes(AxisRef(motor_state=MOTOR_OFF)))
        print("[foctive_controller] quitting")
        break
    elif cmd == "f":
        node.send_output("motor_commands", axis_ref_bytes(AxisRef(motor_state=MOTOR_OFF)))
        print("sent: SERVO OFF")
    elif cmd == "v":
        if len(parts) != 4:
            print("usage: v <volt_d> <volt_q> <vir_ang_freq>")
            continue
        try:
            volt_d, volt_q, vir_ang_freq = (float(parts[1]), float(parts[2]),
                                            float(parts[3]))
        except ValueError:
            print("数値で入力してください")
            continue
        # VOLTAGE: ref_val=volt_d, ref_val_1=volt_q, ref_val_2=vir_ang_freq
        rec = AxisRef(motor_state=MOTOR_VOLTAGE,
                      ref_val=volt_d, ref_val_1=volt_q, ref_val_2=vir_ang_freq,
                      kp_scale=1.0, kv_scale=1.0)
        node.send_output("motor_commands", axis_ref_bytes(rec))
        print(f"sent: VOLTAGE volt_d={volt_d} volt_q={volt_q} "
              f"vir_ang_freq={vir_ang_freq}")
    elif cmd == "setting":
        if len(parts) < 2 or parts[1].lower() not in SETTINGS_CMD:
            print("usage: setting <" + "|".join(SETTINGS_CMD) + "> [args]")
            continue
        sub = parts[1].lower()
        scmd = SETTINGS_CMD[sub]

        if sub == "pread":
            if len(parts) != 3:
                print("usage: setting pread <param_index>")
                continue
            try:
                index = int(parts[2])
            except ValueError:
                print("数値で入力してください")
                continue
            req = SettingsRequest(device_id=DEVICE_ID, cmd=scmd,
                                  param_index=index, value=0)
            node.send_output("settings_request",
                             pa.array(list(pack_settings_request(req)), type=pa.uint8()))
            # 返信(settings_result)を待つ
            ev = node.next(timeout=0.5)
            if ev is not None and ev["type"] == "INPUT" and ev["id"] == "settings_result":
                res = unpack_settings_result(bytes(ev["value"].to_pylist()))
                if res.ok:
                    print(f"param[{res.param_index}] = "
                          f"{interpret_param(res.param_index, res.value)}")
                else:
                    print(f"param[{index}] read FAILED (timeout/error/unsupported)")
            else:
                print("no reply (timeout)")
        elif sub == "readall":
            req = SettingsRequest(device_id=DEVICE_ID, cmd=scmd,
                                  param_index=0, value=0)
            node.send_output("settings_request",
                             pa.array(list(pack_settings_request(req)), type=pa.uint8()))
            # 成功時は param_dump(値) → settings_result(ok) の順で届く
            ev = node.next(timeout=1.0)
            if ev is not None and ev["type"] == "INPUT" and ev["id"] == "param_dump":
                ps = unpack_param_scalars(bytes(ev["value"].to_pylist()))
                print("[readall] 全パラメータ:")
                for name in ps._fields:
                    if name == "elec_angle_ofs":
                        continue  # LUT は下でまとめて表示
                    raw_u32 = getattr(ps, name)
                    if name in INT_PARAM_FIELDS:
                        print(f"  {name:14s} = {raw_u32}")
                    else:
                        f = struct.unpack("<f", struct.pack("<I", raw_u32))[0]
                        print(f"  {name:14s} = {f:g}")
                # 電気角オフセット LUT (64要素) を 8個ずつ折り返し表示
                print("  elec_angle_ofs[64]:")
                lut = ps.elec_angle_ofs
                for r in range(0, len(lut), 8):
                    row = " ".join(f"{v:>10}" for v in lut[r:r + 8])
                    print(f"    [{r:2d}] {row}")
                node.next(timeout=0.2)  # 後続の settings_result を drain
            elif ev is not None and ev["id"] == "settings_result":
                print("readall FAILED (timeout/error on CAN)")
            else:
                print("no reply (timeout)")
        elif sub == "pwrite":
            if len(parts) != 4:
                print("usage: setting pwrite <param_index> <value>")
                continue
            try:
                index = int(parts[2])
            except ValueError:
                print("param_index は整数で")
                continue
            # 値 → 生 4byte (index 0-6 は uint32, それ以外は float ビット)
            try:
                if index in INT_PARAM_INDICES:
                    raw = int(float(parts[3])) & 0xFFFFFFFF
                else:
                    raw = struct.unpack("<I", struct.pack("<f", float(parts[3])))[0]
            except ValueError:
                print("数値で入力してください")
                continue
            req = SettingsRequest(device_id=DEVICE_ID, cmd=scmd,
                                  param_index=index, value=raw)
            node.send_output("settings_request",
                             pa.array(list(pack_settings_request(req)), type=pa.uint8()))
            ev = node.next(timeout=0.5)
            if ev is not None and ev["type"] == "INPUT" and ev["id"] == "settings_result":
                res = unpack_settings_result(bytes(ev["value"].to_pylist()))
                if res.ok:
                    before = interpret_param(res.param_index, res.old_value)
                    after = interpret_param(res.param_index, res.value)
                    print(f"param[{res.param_index}] {before} -> {after}")
                else:
                    print(f"param[{index}] write FAILED (timeout/error/LUT不可)")
            else:
                print("no reply (timeout)")
        else:
            print(f"setting {sub} (cmd={scmd}) は未配線です")
    else:
        print("unknown command")
