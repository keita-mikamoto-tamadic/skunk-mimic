"""MuJoCo simulation backend.

Receives motor_commands (AxisRef[]), runs physics step,
outputs motor_status (AxisAct[]) + imu_data (ImuData).

Reuses MuJoCoSim class from mujoco_node for simulation logic.
"""

import os
import struct
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import pyarrow as pa
from dora import Node

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
MODEL_PATH = os.path.join(
    PROJECT_ROOT, "sim", os.environ.get("MUJOCO_MODEL", "mimic_v2.xml")
)

# ---------------------------------------------------------------------------
# Binary formats (must match C++ structs in shm_data_format.hpp)
# ---------------------------------------------------------------------------
AXIS_REF_FMT = "<B7xdddddd"
AXIS_REF_SIZE = struct.calcsize(AXIS_REF_FMT)  # 56

AXIS_ACT_FMT = "<dddB7x"
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32

IMU_DATA_FMT = "<14d"
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FMT)  # 112

# ---------------------------------------------------------------------------
# MotorState enum (matches enum_def.hpp)
# ---------------------------------------------------------------------------
MOTOR_OFF = 0
MOTOR_STOP = 1
MOTOR_POSITION = 2
MOTOR_VELOCITY = 3
MOTOR_TORQUE = 4
MOTOR_SET_POSITION = 5

# ---------------------------------------------------------------------------
# Joint mapping
# ---------------------------------------------------------------------------
AXIS_JOINT_NAMES = [
    "upper_link_R_joint",
    "lower_link_R_joint",
    "wheel_R_joint",
    "upper_link_L_joint",
    "lower_link_L_joint",
    "wheel_L_joint",
]

AXIS_ACTUATOR_NAMES = [
    "hip_pitch_r",
    "knee_r",
    "wheel_r",
    "hip_pitch_l",
    "knee_l",
    "wheel_l",
]

NUM_AXES = 6
SUBSTEPS = 3


def quat_to_euler(w, x, y, z):
    """Quaternion (w,x,y,z) to (roll, pitch, yaw) in radians."""
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arctan2(2.0 * (w * y - z * x), 1.0 - 2.0 * (y * y + z * z))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class MuJoCoSim:
    def __init__(self, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.joint_ids = []
        self.qpos_addrs = []
        self.dof_addrs = []
        self.actuator_ids = []
        for i in range(NUM_AXES):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, AXIS_JOINT_NAMES[i])
            assert jid >= 0, f"Joint '{AXIS_JOINT_NAMES[i]}' not found in model"
            self.joint_ids.append(jid)
            self.qpos_addrs.append(self.model.jnt_qposadr[jid])
            self.dof_addrs.append(self.model.jnt_dofadr[jid])

            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, AXIS_ACTUATOR_NAMES[i])
            assert aid >= 0, f"Actuator '{AXIS_ACTUATOR_NAMES[i]}' not found in model"
            self.actuator_ids.append(aid)

        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "standing")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
            print(f"Loaded keyframe 'standing'")

        mujoco.mj_forward(self.model, self.data)

        self.base_kp = []
        self.base_kv = []
        for i in range(NUM_AXES):
            aid = self.actuator_ids[i]
            bp = self.model.actuator_biasprm[aid]
            self.base_kp.append(-float(bp[1]))
            self.base_kv.append(-float(bp[2]))

        self.hold_positions = [
            self.data.qpos[self.qpos_addrs[i]] for i in range(NUM_AXES)
        ]

        self.viewer = None
        self._viewer_lock = threading.Lock()

    def launch_viewer(self):
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        print("Viewer launched")

    def _set_actuator_mode(self, axis_idx, motor_state, kp_scale, kv_scale, torque_limit):
        aid = self.actuator_ids[axis_idx]
        kp_s = kp_scale if kp_scale > 0 else 1.0
        kv_s = kv_scale if kv_scale > 0 else 1.0
        base_kp = self.base_kp[axis_idx]
        base_kv = self.base_kv[axis_idx]

        if motor_state == MOTOR_POSITION or motor_state == MOTOR_STOP:
            kp = base_kp * kp_s
            kv = base_kv * kv_s
            self.model.actuator_gainprm[aid, 0] = kp
            self.model.actuator_biasprm[aid, :3] = [0.0, -kp, -kv]
        elif motor_state == MOTOR_VELOCITY:
            kv = base_kv * kv_s
            self.model.actuator_gainprm[aid, 0] = kv
            self.model.actuator_biasprm[aid, :3] = [0.0, 0.0, -kv]
        elif motor_state == MOTOR_TORQUE:
            self.model.actuator_gainprm[aid, 0] = 1.0
            self.model.actuator_biasprm[aid, :3] = [0.0, 0.0, 0.0]
        else:
            self.model.actuator_gainprm[aid, 0] = 0.0
            self.model.actuator_biasprm[aid, :3] = [0.0, 0.0, 0.0]

        if torque_limit > 0 and not np.isnan(torque_limit):
            self.model.actuator_forcerange[aid] = [-torque_limit, torque_limit]

    def apply_commands(self, commands):
        all_off = True
        for i, cmd in enumerate(commands):
            motor_state, ref_val, kp_scale, kv_scale, vel_limit, accel_limit, torque_limit = cmd
            aid = self.actuator_ids[i]

            self._set_actuator_mode(i, motor_state, kp_scale, kv_scale, torque_limit)

            if motor_state == MOTOR_VELOCITY and abs(ref_val) > 1e-6:
                all_off = False
            elif motor_state == MOTOR_TORQUE:
                all_off = False

            if motor_state == MOTOR_OFF:
                self.data.ctrl[aid] = 0.0
            elif motor_state == MOTOR_STOP:
                self.data.ctrl[aid] = self.hold_positions[i]
            elif motor_state == MOTOR_POSITION:
                self.data.ctrl[aid] = ref_val
            elif motor_state == MOTOR_VELOCITY:
                self.data.ctrl[aid] = ref_val
            elif motor_state == MOTOR_TORQUE:
                self.data.ctrl[aid] = ref_val
            elif motor_state == MOTOR_SET_POSITION:
                self.data.qpos[self.qpos_addrs[i]] = ref_val
                self.hold_positions[i] = ref_val
                self.data.ctrl[aid] = 0.0
            else:
                self.data.ctrl[aid] = 0.0

            if motor_state not in (MOTOR_OFF, MOTOR_SET_POSITION):
                self.hold_positions[i] = self.data.qpos[self.qpos_addrs[i]]

        return not all_off

    def step(self, n=SUBSTEPS):
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def forward(self):
        mujoco.mj_forward(self.model, self.data)

    def get_motor_status(self) -> bytes:
        buf = bytearray()
        for i in range(NUM_AXES):
            pos = self.data.qpos[self.qpos_addrs[i]]
            vel = self.data.qvel[self.dof_addrs[i]]
            torque = self.data.actuator_force[self.actuator_ids[i]]
            fault = 0
            buf += struct.pack(AXIS_ACT_FMT, pos, vel, torque, fault)
        return bytes(buf)

    def get_imu_data(self) -> bytes:
        sensor = {}
        for i in range(self.model.nsensor):
            name = self.model.sensor(i).name
            dim = self.model.sensor(i).dim[0]
            start = self.model.sensor(i).adr[0]
            sensor[name] = self.data.sensordata[start : start + dim]

        accel = sensor.get("accel", np.zeros(3))
        gyro = sensor.get("gyro", np.zeros(3))
        quat = sensor.get("imu_quat", np.array([1.0, 0.0, 0.0, 0.0]))

        w, x, y, z = quat
        roll, pitch, yaw = quat_to_euler(w, x, y, z)

        timestamp = self.data.time
        return struct.pack(
            IMU_DATA_FMT,
            timestamp,
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2],
            w, x, y, z,
            roll, pitch, yaw,
        )

    def sync_viewer(self):
        if self.viewer is not None:
            with self._viewer_lock:
                self.viewer.sync()


def main():
    print(f"Loading model: {MODEL_PATH}")
    sim = MuJoCoSim(MODEL_PATH)
    print(f"Model loaded (timestep={sim.model.opt.timestep}s)")

    if os.environ.get("MUJOCO_VIEWER", "0") == "1":
        sim.launch_viewer()

    node = Node("mujoco_backend")

    last_viewer_sync = 0.0
    VIEWER_SYNC_INTERVAL = 1.0 / 60.0

    # コマンドバッファ
    latest_commands = None
    has_new_commands = False

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "motor_commands":
                # motor_commands (AxisRef[]) をバッファ
                raw = bytes(event["value"].to_pylist())
                num_axes = len(raw) // AXIS_REF_SIZE
                latest_commands = []
                for i in range(min(num_axes, NUM_AXES)):
                    cmd = struct.unpack_from(AXIS_REF_FMT, raw, i * AXIS_REF_SIZE)
                    latest_commands.append(cmd)
                has_new_commands = True

            elif event["id"] == "tick":
                # tick 駆動: コマンド適用 → シミュレーション → motor_status 出力
                if has_new_commands and latest_commands is not None:
                    should_step = sim.apply_commands(latest_commands)
                    has_new_commands = False
                    if should_step:
                        sim.step()
                    else:
                        sim.forward()
                else:
                    sim.forward()

                # motor_status (AxisAct[])
                motor_bytes = sim.get_motor_status()
                node.send_output(
                    "motor_status",
                    pa.array(list(motor_bytes), type=pa.uint8()),
                )

                # imu_data
                imu_bytes = sim.get_imu_data()
                node.send_output(
                    "imu_data",
                    pa.array(list(imu_bytes), type=pa.uint8()),
                )

                # Viewer sync
                if sim.viewer is not None:
                    now = time.monotonic()
                    if now - last_viewer_sync >= VIEWER_SYNC_INTERVAL:
                        sim.sync_viewer()
                        last_viewer_sync = now

    print("Shutting down")


if __name__ == "__main__":
    main()
