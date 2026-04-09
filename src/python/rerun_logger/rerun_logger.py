"""Rerun Logger — Dora node that records robot state to .rrd files.

Subscribes to motor_status, motor_commands, imu_data, estimated_state,
state_status and writes to a Rerun .rrd file for post-hoc visualization.
Keeps only the last 30 seconds of data (ring buffer via Rerun memory limit).
"""

from dora import Node
import struct
import signal
import sys
import os
import time
from datetime import datetime

import rerun as rr

# lib を import パスに追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib import robot_config

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "robot_config", "mimic_v2.json")
LOGS_DIR = os.path.join(PROJECT_ROOT, "logs")

# Struct formats (must match shm_data_format.hpp)
AXIS_ACT_FMT = "<dddB7x"
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32

AXIS_REF_FMT = "<B7xdddddd"
AXIS_REF_SIZE = struct.calcsize(AXIS_REF_FMT)  # 56

IMU_DATA_FMT = "<14d"
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FMT)  # 112

EST_STATE_FMT = "<3d"
EST_STATE_SIZE = struct.calcsize(EST_STATE_FMT)  # 24

STATE_NAMES = {0: "OFF", 1: "STOP", 2: "READY", 3: "RUN"}

# 30秒 × 333Hz ≈ 10000 ticks
MEMORY_LIMIT = "50MB"


def main():
    config = robot_config.load_from_file(CONFIG_PATH)
    print(f"Loaded config: {config.robot_name} ({len(config.axes)} axes)")

    # wheel axis indices
    wheel_indices = {}
    for i, ax in enumerate(config.axes):
        if ax.name in ("wheel_r", "wheel_l"):
            wheel_indices[ax.name] = i

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rrd_path = os.path.join(LOGS_DIR, f"{timestamp}.rrd")

    rr.init("skunk_mimic", spawn=False)
    rr.save(rrd_path, default_blueprint=None)
    print(f"Recording to: {rrd_path}")

    node = Node("rerun_logger")
    tick = 0
    running = True

    def on_signal(signum, frame):
        nonlocal running
        running = False
        print(f"\nShutting down (signal {signum})")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    for event in node:
        if not running:
            break
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        raw = bytes(event["value"].to_pylist())
        rr.set_time("tick", sequence=tick)
        tick += 1

        if eid == "imu_data" and len(raw) >= IMU_DATA_SIZE:
            vals = struct.unpack(IMU_DATA_FMT, raw[:IMU_DATA_SIZE])
            # timestamp, ax, ay, az, gx, gy, gz, q0-q3, roll, pitch, yaw
            rr.log("imu/ax", rr.Scalars(vals[1]))
            rr.log("imu/ay", rr.Scalars(vals[2]))
            rr.log("imu/az", rr.Scalars(vals[3]))
            rr.log("imu/gx", rr.Scalars(vals[4]))
            rr.log("imu/gy", rr.Scalars(vals[5]))
            rr.log("imu/gz", rr.Scalars(vals[6]))
            rr.log("imu/roll", rr.Scalars(vals[11]))
            rr.log("imu/pitch", rr.Scalars(vals[12]))
            rr.log("imu/yaw", rr.Scalars(vals[13]))

        elif eid == "motor_status":
            n_axes = len(raw) // AXIS_ACT_SIZE
            for i in range(n_axes):
                pos, vel, torq, fault = struct.unpack_from(
                    AXIS_ACT_FMT, raw, i * AXIS_ACT_SIZE
                )
                name = config.axes[i].name if i < len(config.axes) else f"axis{i}"
                rr.log(f"motor/{name}/position", rr.Scalars(pos))
                rr.log(f"motor/{name}/velocity", rr.Scalars(vel))
                rr.log(f"motor/{name}/torque", rr.Scalars(torq))

        elif eid == "motor_commands":
            n_axes = len(raw) // AXIS_REF_SIZE
            for i in range(n_axes):
                state, ref, kp, kv, vlim, alim, tlim = struct.unpack_from(
                    AXIS_REF_FMT, raw, i * AXIS_REF_SIZE
                )
                name = config.axes[i].name if i < len(config.axes) else f"axis{i}"
                rr.log(f"cmd/{name}/ref_val", rr.Scalars(ref))
                rr.log(f"cmd/{name}/motor_state", rr.Scalars(state))

        elif eid == "estimated_state" and len(raw) >= EST_STATE_SIZE:
            vel, yaw, yaw_rate = struct.unpack(EST_STATE_FMT, raw[:EST_STATE_SIZE])
            rr.log("ekf/velocity", rr.Scalars(vel))
            rr.log("ekf/yaw", rr.Scalars(yaw))
            rr.log("ekf/yaw_rate", rr.Scalars(yaw_rate))

        elif eid == "state_status" and len(raw) >= 1:
            state_val = raw[0]
            rr.log("state", rr.TextLog(STATE_NAMES.get(state_val, f"?{state_val}")))

    print(f"Saved: {rrd_path}")


if __name__ == "__main__":
    main()
