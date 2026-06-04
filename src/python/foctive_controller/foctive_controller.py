from dora import Node
import pyarrow as pa
import sys
import os

# lib を import パスに追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.data_format import AxisRef, pack_axis_ref  # 自動生成(axis_data.json 正本)

# MotorState (enum_def.hpp と一致)
MOTOR_OFF = 0
MOTOR_VOLTAGE = 7


def axis_ref_bytes(rec):
    # AxisRef(namedtuple) → uint8 配列 (dora 送信用)
    return pa.array(list(pack_axis_ref(rec)), type=pa.uint8())


node = Node("foctive_controller")

print("[foctive_controller] Commands:")
print("  v <volt_d> <volt_q> <vir_ang_freq> : 電圧制御 (例: v 0 1.0 20)")
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
    else:
        print("unknown command")
