from dora import Node
import struct
import time
import sys
import os

# lib を import パスに追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib import robot_config

# Config file path (absolute path computed from script location)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "robot_config", "mimic_v2.json")

# AxisAct: double position, double velocity, double torque, uint8_t fault (+7 padding)
AXIS_ACT_FMT = "<dddB7x"
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32

# ImuData: timestamp, ax, ay, az, gx, gy, gz, q0, q1, q2, q3, roll, pitch, yaw
IMU_DATA_FMT = "<14d"
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FMT)  # 112

# EstimatedState: velocity, yaw, yaw_rate
EST_STATE_FMT = "<3d"
EST_STATE_SIZE = struct.calcsize(EST_STATE_FMT)  # 24

# LatencyData: can_avg_us, can_max_us, ctrl_avg_us, ctrl_max_us
LATENCY_FMT = "<4d"
LATENCY_SIZE = struct.calcsize(LATENCY_FMT)  # 32

STATE_NAMES = {0: "OFF", 1: "STOP", 2: "READY", 3: "RUN"}
STATE_COLORS = {0: "37", 1: "33", 2: "34", 3: "32"}  # white, yellow, blue, green

# ANSI helpers
CSI = "\033["
CLEAR = f"{CSI}2J"
HOME = f"{CSI}H"
RESET = f"{CSI}0m"
BOLD = f"{CSI}1m"
DIM = f"{CSI}2m"


def color(text, code):
    return f"{CSI}{code}m{text}{RESET}"


def latency_color(avg):
    if avg < 1500:
        return "32"  # green
    elif avg < 2500:
        return "33"  # yellow
    return "31"  # red


def render(axes, timestamp_ns, state, imu_data, est_state, latency_data, config):
    lines = []

    # Motor table
    title = config.robot_name if config else "motor_status"
    lines.append(f"{BOLD}{title}{RESET}")
    lines.append(f"{'axis':<16} {'CAN':>4} {'position':>10} {'velocity':>10} {'torque':>9} {'fault':>5}")
    lines.append("-" * 60)

    if axes:
        for i, (pos, vel, torq, fault) in enumerate(axes):
            name = config.axes[i].name if config and i < len(config.axes) else f"#{i}"
            can_id = config.axes[i].device_id if config and i < len(config.axes) else "-"
            fault_str = color(str(fault), "31;1") if fault != 0 else color(str(fault), "32")
            lines.append(f"{color(name, '36'):<25} {can_id:>4} {pos:>+10.4f} {vel:>+10.4f} {torq:>+9.4f} {fault_str:>14}")

    # State + Time
    state_name = STATE_NAMES.get(state, "?") if state is not None else "-"
    state_col = STATE_COLORS.get(state, "37") if state is not None else "37"
    time_str = f"{timestamp_ns / 1e9:.3f}s" if timestamp_ns is not None else "-"
    lines.append(f"State: {color(state_name, state_col + ';1')}  |  Time: {DIM}{time_str}{RESET}")
    lines.append("")

    # IMU / EKF
    lines.append(f"{BOLD}IMU / EKF{RESET}")
    lines.append(f"{'':>4} {'Roll/est_vel':>12} {'Pitch/est_yaw':>13} {'Yaw/est_yawR':>13} {'ax':>7} {'ay':>7} {'az':>7}")
    if imu_data:
        roll, pitch, yaw = imu_data[11], imu_data[12], imu_data[13]
        ax, ay, az = imu_data[1], imu_data[2], imu_data[3]
        lines.append(f"{DIM}IMU{RESET}  {color(f'{roll:+.3f}', '36'):>21} {color(f'{pitch:+.3f}', '36'):>22} {color(f'{yaw:+.3f}', '36'):>22} {ax:>+7.2f} {ay:>+7.2f} {az:>+7.2f}")
    else:
        lines.append(f"{DIM}IMU{RESET}  {'-':>12} {'-':>13} {'-':>13} {'-':>7} {'-':>7} {'-':>7}")

    if est_state:
        vel, yaw_est, yaw_rate = est_state
        lines.append(f"{DIM}EKF{RESET}  {color(f'{vel:+.4f}', '35'):>21} {color(f'{yaw_est:+.3f}', '35'):>22} {color(f'{yaw_rate:+.3f}', '35'):>22}")
    else:
        lines.append(f"{DIM}EKF{RESET}  {'-':>12} {'-':>13} {'-':>13}")
    lines.append("")

    # Latency
    lines.append(f"{BOLD}Latency{RESET}")
    if latency_data:
        can_avg, can_max, ctrl_avg, ctrl_max = latency_data
        total_avg = can_avg + ctrl_avg
        lines.append(f"  CAN   {color(f'{can_avg:.0f}us', latency_color(can_avg)):>18}  max {can_max:.0f}us")
        lines.append(f"  CTRL  {color(f'{ctrl_avg:.0f}us', latency_color(ctrl_avg)):>18}  max {ctrl_max:.0f}us")
        lines.append(f"  Total {color(f'{total_avg:.0f}us', latency_color(total_avg)):>18}")
    else:
        lines.append("  CAN   -")
        lines.append("  CTRL  -")

    return "\n".join(lines)


node = Node("data_viewer")

config = robot_config.load_from_file(CONFIG_PATH)
print(f"Loaded config: {config.robot_name} ({config.axis_count} axes)")

start_time = time.time_ns()
current_state = None
current_imu_data = None
current_latency = None
current_est_state = None
current_axes = None

# 画面クリア
sys.stdout.write(CLEAR)
sys.stdout.flush()

for event in node:
    if event["type"] == "INPUT":
        event_id = event["id"]

        # tick (100ms) で表示更新
        if event_id == "tick":
            timestamp_ns = time.time_ns() - start_time
            output = render(current_axes, timestamp_ns, current_state,
                          current_imu_data, current_est_state, current_latency, config)
            sys.stdout.write(HOME + output + "\n")
            sys.stdout.flush()
            continue

        # データ受信: デコードして保存のみ
        raw = bytes(event["value"].to_pylist())
        if event_id == "state_status":
            if len(raw) > 0:
                current_state = raw[0]
        elif event_id == "imu_data":
            if len(raw) >= IMU_DATA_SIZE:
                current_imu_data = struct.unpack(IMU_DATA_FMT, raw[:IMU_DATA_SIZE])
        elif event_id == "latency":
            if len(raw) >= LATENCY_SIZE:
                current_latency = struct.unpack(LATENCY_FMT, raw[:LATENCY_SIZE])
        elif event_id == "estimated_state":
            if len(raw) >= EST_STATE_SIZE:
                current_est_state = struct.unpack(EST_STATE_FMT, raw[:EST_STATE_SIZE])
        elif event_id == "motor_status":
            axis_count = len(raw) // AXIS_ACT_SIZE
            current_axes = [
                struct.unpack_from(AXIS_ACT_FMT, raw, i * AXIS_ACT_SIZE)
                for i in range(axis_count)
            ]
