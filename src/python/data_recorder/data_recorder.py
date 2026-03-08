"""Data recorder node for system identification.

Records motor_commands, motor_status, and imu_data to .npz files.
Uses time.monotonic() for unified timestamps.
Saves on SIGINT or Dora shutdown.
"""

import os
import signal
import struct
import sys
import time

import numpy as np
import pyarrow as pa
from dora import Node

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "sysid", "data")

# ---------------------------------------------------------------------------
# Binary formats (must match C++ structs in shm_data_format.hpp)
# ---------------------------------------------------------------------------
# AxisRef: MotorState(u8) + pad(7) + 6 doubles
AXIS_REF_FMT = "<B7xdddddd"
AXIS_REF_SIZE = struct.calcsize(AXIS_REF_FMT)  # 56

# AxisAct: 3 doubles + fault(u8) + pad(7)
AXIS_ACT_FMT = "<dddB7x"
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32

# ImuData: 14 doubles
IMU_DATA_FMT = "<14d"
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FMT)  # 112

NUM_AXES = 6


class DataRecorder:
    def __init__(self):
        # Lists to accumulate data (converted to arrays on save)
        self.cmd_timestamps = []
        self.cmd_motor_state = []  # (N, 6) uint8
        self.cmd_ref_val = []      # (N, 6) float64
        self.cmd_kp_scale = []     # (N, 6) float64
        self.cmd_kv_scale = []     # (N, 6) float64
        self.cmd_vel_limit = []    # (N, 6) float64
        self.cmd_accel_limit = []  # (N, 6) float64
        self.cmd_torque_limit = [] # (N, 6) float64

        self.status_timestamps = []
        self.status_position = []   # (N, 6) float64
        self.status_velocity = []   # (N, 6) float64
        self.status_torque = []     # (N, 6) float64
        self.status_fault = []      # (N, 6) uint8

        self.imu_timestamps = []
        self.imu_data = []          # (N, 14) float64

        self._start_time = time.monotonic()

    def record_commands(self, raw: bytes):
        t = time.monotonic() - self._start_time
        num_axes = len(raw) // AXIS_REF_SIZE
        if num_axes < NUM_AXES:
            return
        states, refs, kps, kvs, vls, als, tls = [], [], [], [], [], [], []
        for i in range(NUM_AXES):
            ms, rv, kp, kv, vl, al, tl = struct.unpack_from(
                AXIS_REF_FMT, raw, i * AXIS_REF_SIZE
            )
            states.append(ms)
            refs.append(rv)
            kps.append(kp)
            kvs.append(kv)
            vls.append(vl)
            als.append(al)
            tls.append(tl)
        self.cmd_timestamps.append(t)
        self.cmd_motor_state.append(states)
        self.cmd_ref_val.append(refs)
        self.cmd_kp_scale.append(kps)
        self.cmd_kv_scale.append(kvs)
        self.cmd_vel_limit.append(vls)
        self.cmd_accel_limit.append(als)
        self.cmd_torque_limit.append(tls)

    def record_status(self, raw: bytes):
        t = time.monotonic() - self._start_time
        num_axes = len(raw) // AXIS_ACT_SIZE
        if num_axes < NUM_AXES:
            return
        pos, vel, torq, flt = [], [], [], []
        for i in range(NUM_AXES):
            p, v, tq, f = struct.unpack_from(
                AXIS_ACT_FMT, raw, i * AXIS_ACT_SIZE
            )
            pos.append(p)
            vel.append(v)
            torq.append(tq)
            flt.append(f)
        self.status_timestamps.append(t)
        self.status_position.append(pos)
        self.status_velocity.append(vel)
        self.status_torque.append(torq)
        self.status_fault.append(flt)

    def record_imu(self, raw: bytes):
        t = time.monotonic() - self._start_time
        if len(raw) < IMU_DATA_SIZE:
            return
        data = struct.unpack(IMU_DATA_FMT, raw[:IMU_DATA_SIZE])
        self.imu_timestamps.append(t)
        self.imu_data.append(data)

    def save(self, suffix=""):
        os.makedirs(DATA_DIR, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = f"sysid_{ts}{suffix}.npz"
        filepath = os.path.join(DATA_DIR, filename)

        n_cmd = len(self.cmd_timestamps)
        n_status = len(self.status_timestamps)
        n_imu = len(self.imu_timestamps)

        data = {}
        if n_cmd > 0:
            data["cmd_timestamps"] = np.array(self.cmd_timestamps)
            data["cmd_motor_state"] = np.array(self.cmd_motor_state, dtype=np.uint8)
            data["cmd_ref_val"] = np.array(self.cmd_ref_val)
            data["cmd_kp_scale"] = np.array(self.cmd_kp_scale)
            data["cmd_kv_scale"] = np.array(self.cmd_kv_scale)
            data["cmd_vel_limit"] = np.array(self.cmd_vel_limit)
            data["cmd_accel_limit"] = np.array(self.cmd_accel_limit)
            data["cmd_torque_limit"] = np.array(self.cmd_torque_limit)

        if n_status > 0:
            data["status_timestamps"] = np.array(self.status_timestamps)
            data["status_position"] = np.array(self.status_position)
            data["status_velocity"] = np.array(self.status_velocity)
            data["status_torque"] = np.array(self.status_torque)
            data["status_fault"] = np.array(self.status_fault, dtype=np.uint8)

        if n_imu > 0:
            data["imu_timestamps"] = np.array(self.imu_timestamps)
            data["imu_data"] = np.array(self.imu_data)

        if data:
            np.savez(filepath, **data)
            print(f"[data_recorder] Saved {filepath}")
            print(f"  commands: {n_cmd}, status: {n_status}, imu: {n_imu}")
        else:
            print("[data_recorder] No data to save")

        return filepath


def main():
    recorder = DataRecorder()
    saved = False

    def on_signal(signum, frame):
        nonlocal saved
        if not saved:
            recorder.save()
            saved = True
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    node = Node("data_recorder")
    print(f"[data_recorder] Recording to {DATA_DIR}")

    sample_count = 0
    for event in node:
        if event["type"] == "INPUT":
            raw = bytes(event["value"].to_pylist())
            eid = event["id"]
            if eid == "motor_commands":
                recorder.record_commands(raw)
            elif eid == "motor_status":
                recorder.record_status(raw)
                sample_count += 1
                if sample_count % 1000 == 0:
                    print(f"[data_recorder] {sample_count} status samples")
            elif eid == "imu_data":
                recorder.record_imu(raw)

    # Save on normal Dora shutdown
    if not saved:
        recorder.save()


if __name__ == "__main__":
    main()
