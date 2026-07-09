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
    unpack_param_scalars, PARAM_SCALARS_WIRE_INDEX,
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
    "calib": 1,      # 電気角キャリブ (モータが回り数秒)
    "setpos": 110,   # 現在位置を任意の値として設定
}

# uint32 として解釈するパラメータ index (それ以外は float)。index 8 は LUT(scalar外)
INT_PARAM_INDICES = {0, 1, 2, 3, 4, 5, 6}


def interpret_param(index, value_u32):
    if index in INT_PARAM_INDICES:
        return value_u32
    # float: 生ビットを float として再解釈
    return struct.unpack("<f", struct.pack("<I", value_u32))[0]


def print_param_dump(ps, title):
    # ParamScalars(26 scalar + LUT)を表示。cmd=102/101 共通
    print(title)
    # 行頭の [NN] は wire param_index(pread/pwrite で使う番号)。axis_data.json 正本。
    for name in ps._fields:
        if name == "elec_angle_ofs":
            continue  # LUT は下でまとめて表示
        idx = PARAM_SCALARS_WIRE_INDEX[name]
        raw_u32 = getattr(ps, name)
        if name in INT_PARAM_FIELDS:
            print(f"  [{idx:2d}] {name:18s} = {raw_u32}")
        else:
            f = struct.unpack("<f", struct.pack("<I", raw_u32))[0]
            print(f"  [{idx:2d}] {name:18s} = {f:g}")
    print(f"  [{PARAM_SCALARS_WIRE_INDEX['elec_angle_ofs']:2d}] elec_angle_ofs[64]:")
    lut = ps.elec_angle_ofs
    for r in range(0, len(lut), 8):
        row = " ".join(f"{v:>10}" for v in lut[r:r + 8])
        print(f"    [{r:2d}] {row}")

# MotorState (enum_def.hpp と一致)
MOTOR_OFF = 0
MOTOR_POSITION = 2
MOTOR_VELOCITY = 3
MOTOR_CURRENT = 6
MOTOR_VOLTAGE = 7
MOTOR_POSITION_PD = 8   # インピーダンス制御
MOTOR_CASCADE_POS_PID = 9   # FOCTIVE ネイティブ位置カスケードPID
MOTOR_CASCADE_VEL_PID = 10  # FOCTIVE ネイティブ速度カスケードPID


def axis_ref_bytes(rec):
    # AxisRef(namedtuple) → uint8 配列 (dora 送信用)
    return pa.array(list(pack_axis_ref(rec)), type=pa.uint8())


node = Node("foctive_controller")

DEVICE_ID = 1  # 単軸テスト用

print("[foctive_controller] Commands:")
print("  v <volt_d> <volt_q> <vir_ang_freq> : 電圧制御 (例: v 0 1.0 20)")
print("  c <cur_d> <cur_q>                  : 電流制御 (例: c 0 0.5)")
print("  vel <vel> [kp] [kd] [accel_limit]  : 速度制御(インピーダンス) (例: vel 5.0 / vel 5.0 1 1 20)")
print("  p <pos> [kp] [kd] [accel_limit]    : 位置制御(インピーダンス) (例: p 1.57 / p 1.57 1 1 10)")
print("  pd <pos> <vel> <torq> [kp] [kd]    : インピーダンス制御 (例: pd 1.57 0 0)")
print("  cpos <pos> [accel_limit]           : カスケードPID位置 (FOCTIVE専用)")
print("  cvel <vel> [accel_limit]           : カスケードPID速度 (FOCTIVE専用)")
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
    elif cmd == "c":
        if len(parts) != 3:
            print("usage: c <cur_d> <cur_q>")
            continue
        try:
            cur_d, cur_q = float(parts[1]), float(parts[2])
        except ValueError:
            print("数値で入力してください")
            continue
        # CURRENT: ref_val=cur_d, ref_val_1=cur_q (ToCommand のマッピングに一致)
        rec = AxisRef(motor_state=MOTOR_CURRENT,
                      ref_val=cur_d, ref_val_1=cur_q,
                      kp_scale=1.0, kv_scale=1.0)
        node.send_output("motor_commands", axis_ref_bytes(rec))
        print(f"sent: CURRENT cur_d={cur_d} cur_q={cur_q}")
    elif cmd == "vel":
        if len(parts) not in (2, 4, 5):
            print("usage: vel <vel> [kp] [kd] [accel_limit]")
            continue
        try:
            vel = float(parts[1])
            kp = float(parts[2]) if len(parts) >= 4 else 1.0
            kd = float(parts[3]) if len(parts) >= 4 else 1.0
            accel = float(parts[4]) if len(parts) == 5 else 0.0
        except ValueError:
            print("数値で入力してください")
            continue
        # VELOCITY → インピーダンス(pos=NaN): kp=前進目標の追従(=速度誤差積分),
        #   kd=速度ダンピング。kp=0 で純速度。
        rec = AxisRef(motor_state=MOTOR_VELOCITY, ref_val=vel, accel_limit=accel,
                      kp_scale=kp, kv_scale=kd)
        node.send_output("motor_commands", axis_ref_bytes(rec))
        print(f"sent: VELOCITY vel={vel} kp={kp} kd={kd} accel_limit={accel}")
    elif cmd == "p":
        if len(parts) not in (2, 4, 5):
            print("usage: p <pos> [kp] [kd] [accel_limit]")
            continue
        try:
            pos = float(parts[1])
            kp = float(parts[2]) if len(parts) >= 4 else 1.0
            kd = float(parts[3]) if len(parts) >= 4 else 1.0
            accel = float(parts[4]) if len(parts) == 5 else 0.0
        except ValueError:
            print("数値で入力してください")
            continue
        # POSITION → インピーダンス: pos 目標 + kp/kd スケール + accel_limit(任意)
        rec = AxisRef(motor_state=MOTOR_POSITION, ref_val=pos, accel_limit=accel,
                      kp_scale=kp, kv_scale=kd)
        node.send_output("motor_commands", axis_ref_bytes(rec))
        print(f"sent: POSITION pos={pos} kp={kp} kd={kd} accel_limit={accel}")
    elif cmd == "pd":
        if len(parts) not in (4, 6):
            print("usage: pd <pos> <vel> <torq> [kp_scale] [kd_scale]  (インピーダンス)")
            continue
        try:
            pos, vel, torq = float(parts[1]), float(parts[2]), float(parts[3])
            kp = float(parts[4]) if len(parts) == 6 else 1.0
            kd = float(parts[5]) if len(parts) == 6 else 1.0
        except ValueError:
            print("数値で入力してください")
            continue
        # POSITION_PD: ref_val=pos, ref_val_1=vel, ref_val_2=torq(FFトルク),
        #   kp_scale/kv_scale = パラメータ imp_kp/imp_kd へのスケール (1.0=フルゲイン)
        rec = AxisRef(motor_state=MOTOR_POSITION_PD,
                      ref_val=pos, ref_val_1=vel, ref_val_2=torq,
                      kp_scale=kp, kv_scale=kd)
        node.send_output("motor_commands", axis_ref_bytes(rec))
        print(f"sent: IMPEDANCE pos={pos} vel={vel} torq={torq} kp_scale={kp} kd_scale={kd}")
    elif cmd == "cpos":
        if len(parts) not in (2, 3):
            print("usage: cpos <pos> [accel_limit]  (カスケードPID位置, FOCTIVE専用)")
            continue
        try:
            pos = float(parts[1])
            accel = float(parts[2]) if len(parts) == 3 else 0.0
        except ValueError:
            print("数値で入力してください")
            continue
        rec = AxisRef(motor_state=MOTOR_CASCADE_POS_PID, ref_val=pos, accel_limit=accel,
                      kp_scale=1.0, kv_scale=1.0)
        node.send_output("motor_commands", axis_ref_bytes(rec))
        print(f"sent: CASCADE_POS pos={pos} accel_limit={accel}")
    elif cmd == "cvel":
        if len(parts) not in (2, 3):
            print("usage: cvel <vel> [accel_limit]  (カスケードPID速度, FOCTIVE専用)")
            continue
        try:
            vel = float(parts[1])
            accel = float(parts[2]) if len(parts) == 3 else 0.0
        except ValueError:
            print("数値で入力してください")
            continue
        rec = AxisRef(motor_state=MOTOR_CASCADE_VEL_PID, ref_val=vel, accel_limit=accel,
                      kp_scale=1.0, kv_scale=1.0)
        node.send_output("motor_commands", axis_ref_bytes(rec))
        print(f"sent: CASCADE_VEL vel={vel} accel_limit={accel}")
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
                print_param_dump(ps, "[readall] 全パラメータ:")
                node.next(timeout=0.2)  # 後続の settings_result を drain
            elif ev is not None and ev["type"] == "INPUT" and ev["id"] == "settings_result":
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
        elif sub == "save":
            req = SettingsRequest(device_id=DEVICE_ID, cmd=scmd,
                                  param_index=0, value=0)
            node.send_output("settings_request",
                             pa.array(list(pack_settings_request(req)), type=pa.uint8()))
            # EEPROM 書込で時間がかかるので長めに待つ
            ev = node.next(timeout=1.0)
            if ev is not None and ev["type"] == "INPUT" and ev["id"] == "settings_result":
                res = unpack_settings_result(bytes(ev["value"].to_pylist()))
                print("saved (EEPROM)" if res.ok else "save FAILED (timeout/error)")
            else:
                print("no reply (timeout)")
        elif sub == "load":
            req = SettingsRequest(device_id=DEVICE_ID, cmd=scmd,
                                  param_index=0, value=0)
            node.send_output("settings_request",
                             pa.array(list(pack_settings_request(req)), type=pa.uint8()))
            # 返信は初期値(readall と同形)
            ev = node.next(timeout=1.0)
            if ev is not None and ev["type"] == "INPUT" and ev["id"] == "param_dump":
                ps = unpack_param_scalars(bytes(ev["value"].to_pylist()))
                print_param_dump(ps, "[load] 初期値ロード後:")
                node.next(timeout=0.2)  # settings_result を drain
            elif ev is not None and ev["type"] == "INPUT" and ev["id"] == "settings_result":
                print("load FAILED (timeout/error on CAN)")
            else:
                print("no reply (timeout)")
        elif sub == "calib":
            if len(parts) != 3:
                print("usage: setting calib <volt_d>  (例: setting calib 1.0)")
                continue
            try:
                volt_d = float(parts[2])
            except ValueError:
                print("数値で入力してください")
                continue
            # volt_d を float ビットで value に載せる
            raw = struct.unpack("<I", struct.pack("<f", volt_d))[0]
            req = SettingsRequest(device_id=DEVICE_ID, cmd=scmd,
                                  param_index=0, value=raw)
            node.send_output("settings_request",
                             pa.array(list(pack_settings_request(req)), type=pa.uint8()))
            print(f"calibrating (volt_d={volt_d}) ... モータが回ります。完了まで数秒")
            # ファーム側 timeout(10s)より長く待つ
            ev = node.next(timeout=12.0)
            if ev is not None and ev["type"] == "INPUT" and ev["id"] == "settings_result":
                res = unpack_settings_result(bytes(ev["value"].to_pylist()))
                if res.ok:
                    pos = struct.unpack("<f", struct.pack("<I", res.value))[0]
                    print(f"calibration DONE (pos={pos:g})")
                else:
                    print("calibration FAILED (timeout/error)")
            else:
                print("no reply (timeout)")
        elif sub == "setpos":
            if len(parts) != 3:
                print("usage: setting setpos <value>  "
                      "(現在位置をこの値として設定。例: setting setpos 0)")
                continue
            try:
                target = float(parts[2])
            except ValueError:
                print("数値で入力してください")
                continue
            # 設定したい値を float ビットで value に載せる
            raw = struct.unpack("<I", struct.pack("<f", target))[0]
            req = SettingsRequest(device_id=DEVICE_ID, cmd=scmd,
                                  param_index=0, value=raw)
            node.send_output("settings_request",
                             pa.array(list(pack_settings_request(req)), type=pa.uint8()))
            ev = node.next(timeout=0.5)
            if ev is not None and ev["type"] == "INPUT" and ev["id"] == "settings_result":
                res = unpack_settings_result(bytes(ev["value"].to_pylist()))
                if res.ok:
                    offset = struct.unpack("<f", struct.pack("<I", res.value))[0]
                    print(f"setpos: 現在位置 -> {target:g} (offset={offset:g})")
                else:
                    print("setpos FAILED (timeout/error)")
            else:
                print("no reply (timeout)")
        else:
            print(f"setting {sub} (cmd={scmd}) は未配線です")
    else:
        print("unknown command")
