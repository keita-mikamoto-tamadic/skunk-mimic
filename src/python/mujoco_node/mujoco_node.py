"""MuJoCo simulation node for Dora.

Replaces device_control_manager + imu_node.
Receives motor_commands (AxisRef[6]), runs physics step,
outputs motor_status (AxisAct[6]) + imu_data (ImuData).

Actuator model:
  Uses MuJoCo general actuators with runtime gainprm/biasprm switching.
  Force = gainprm[0]*ctrl + biasprm[0] + biasprm[1]*qpos + biasprm[2]*qvel
    POSITION: gain=kp, bias=[0, -kp, -kv] → kp*(ctrl-qpos) - kv*qvel
    VELOCITY: gain=kv, bias=[0,   0, -kv] → kv*(ctrl-qvel)
    TORQUE:   gain=1,  bias=[0,   0,   0] → ctrl
    OFF:      gain=0,  bias=[0,   0,   0] → 0
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
# AxisRef: MotorState(u8) + pad(7) + 6 doubles
AXIS_REF_FMT = "<B7xdddddd"
AXIS_REF_SIZE = struct.calcsize(AXIS_REF_FMT)  # 56

# AxisAct: 3 doubles + fault(u8) + pad(7)
AXIS_ACT_FMT = "<dddB7x"
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32

# ImuData: 14 doubles
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
# Joint mapping: Dora axis index → MJCF names
# robot_config order: hip_pitch_r(0), knee_r(1), wheel_r(2),
#                     hip_pitch_l(3), knee_l(4), wheel_l(5)
# ---------------------------------------------------------------------------
AXIS_JOINT_NAMES = [
    "upper_link_R_joint",  # 0: hip_pitch_r
    "lower_link_R_joint",  # 1: knee_r
    "wheel_R_joint",       # 2: wheel_r
    "upper_link_L_joint",  # 3: hip_pitch_l
    "lower_link_L_joint",  # 4: knee_l
    "wheel_L_joint",       # 5: wheel_l
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

# Number of physics substeps per control tick.
# Control tick = 3ms, physics timestep = 1ms → 3 substeps.
SUBSTEPS = 3

# Base servo gains (matching moteus PID: kp=140 Nm/rev, kd=0.5 Nm/(rev/s))
# Convert from Nm/rev to Nm/rad: divide by 2π
MUJOCOSIM_GAIN_KP = 1.4
BASE_KP = MUJOCOSIM_GAIN_KP * 140.0 / (2.0 * 3.141592653589793)   # ≈ 22.28 Nm/rad
BASE_KV = 0.5 / (2.0 * 3.141592653589793)     # ≈ 0.0796 Nm/(rad/s)


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

        # Cache joint / actuator ids
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

        # Load standing keyframe
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "standing")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
            print(f"[mujoco_node] Loaded keyframe 'standing'")

        # Initialize physics state (contacts, sensors) without stepping dynamics
        mujoco.mj_forward(self.model, self.data)

        # Per-axis hold position (for STOP mode) — init from keyframe
        self.hold_positions = [
            self.data.qpos[self.qpos_addrs[i]] for i in range(NUM_AXES)
        ]

        # Viewer (optional)
        self.viewer = None
        self._viewer_lock = threading.Lock()

    def launch_viewer(self):
        """Launch passive viewer in a background thread."""
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        print("[mujoco_node] Viewer launched")

    def _set_actuator_mode(self, axis_idx, motor_state, kp_scale, kv_scale, torque_limit):
        """Configure MuJoCo general actuator gainprm/biasprm for the given mode."""
        aid = self.actuator_ids[axis_idx]
        kp_s = kp_scale if kp_scale > 0 else 1.0
        kv_s = kv_scale if kv_scale > 0 else 1.0

        if motor_state == MOTOR_POSITION or motor_state == MOTOR_STOP:
            kp = BASE_KP * kp_s
            kv = BASE_KV * kv_s
            self.model.actuator_gainprm[aid, 0] = kp
            self.model.actuator_biasprm[aid, :3] = [0.0, -kp, -kv]
        elif motor_state == MOTOR_VELOCITY:
            kv = BASE_KV * kv_s
            self.model.actuator_gainprm[aid, 0] = kv
            self.model.actuator_biasprm[aid, :3] = [0.0, 0.0, -kv]
        elif motor_state == MOTOR_TORQUE:
            self.model.actuator_gainprm[aid, 0] = 1.0
            self.model.actuator_biasprm[aid, :3] = [0.0, 0.0, 0.0]
        else:  # OFF, SET_POSITION, unknown
            self.model.actuator_gainprm[aid, 0] = 0.0
            self.model.actuator_biasprm[aid, :3] = [0.0, 0.0, 0.0]

        # Update forcerange (torque limit)
        if torque_limit > 0 and not np.isnan(torque_limit):
            self.model.actuator_forcerange[aid] = [-torque_limit, torque_limit]

    def apply_commands(self, commands):
        """Apply AxisRef commands to actuators.

        commands: list of (motor_state, ref_val, kp_scale, kv_scale,
                          velocity_limit, accel_limit, torque_limit)
        Returns True if physics should step (active control detected),
        False if frozen (simulates operator holding the robot).
        """
        all_off = True
        for i, cmd in enumerate(commands):
            motor_state, ref_val, kp_scale, kv_scale, vel_limit, accel_limit, torque_limit = cmd
            aid = self.actuator_ids[i]

            # Configure actuator model for this mode
            self._set_actuator_mode(i, motor_state, kp_scale, kv_scale, torque_limit)

            # Unfreeze when active control is commanded:
            # - VELOCITY with nonzero target (= RUN state balance control)
            # - TORQUE mode
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
                # Directly set qpos (encoder reset equivalent)
                self.data.qpos[self.qpos_addrs[i]] = ref_val
                self.hold_positions[i] = ref_val
                self.data.ctrl[aid] = 0.0
            else:
                self.data.ctrl[aid] = 0.0

            # Update hold position for STOP mode
            if motor_state not in (MOTOR_OFF, MOTOR_SET_POSITION):
                self.hold_positions[i] = self.data.qpos[self.qpos_addrs[i]]

        return not all_off  # True = should step physics

    def step(self, n=SUBSTEPS):
        """Advance physics by n substeps (default: SUBSTEPS per control tick)."""
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def forward(self):
        """Compute forward kinematics/sensors without stepping dynamics.

        Called during freeze to keep sensor data up to date.
        """
        mujoco.mj_forward(self.model, self.data)

    def get_motor_status(self) -> bytes:
        """Pack AxisAct[6] as raw bytes."""
        buf = bytearray()
        for i in range(NUM_AXES):
            pos = self.data.qpos[self.qpos_addrs[i]]
            vel = self.data.qvel[self.dof_addrs[i]]
            # Read actual actuator force (after MuJoCo computation)
            torque = self.data.actuator_force[self.actuator_ids[i]]
            fault = 0
            buf += struct.pack(AXIS_ACT_FMT, pos, vel, torque, fault)
        return bytes(buf)

    def get_imu_data(self) -> bytes:
        """Pack ImuData as raw bytes, reading from MuJoCo sensors."""
        sensor = {}
        for i in range(self.model.nsensor):
            name = self.model.sensor(i).name
            dim = self.model.sensor(i).dim[0]
            start = self.model.sensor(i).adr[0]
            sensor[name] = self.data.sensordata[start : start + dim]

        accel = sensor.get("accel", np.zeros(3))
        gyro = sensor.get("gyro", np.zeros(3))
        quat = sensor.get("imu_quat", np.array([1.0, 0.0, 0.0, 0.0]))

        # quat: MuJoCo returns (w, x, y, z)
        w, x, y, z = quat
        roll, pitch, yaw = quat_to_euler(w, x, y, z)

        timestamp = self.data.time
        return struct.pack(
            IMU_DATA_FMT,
            timestamp,
            accel[0], accel[1], accel[2],  # ax, ay, az
            gyro[0], gyro[1], gyro[2],     # gx, gy, gz
            w, x, y, z,                     # q0, q1, q2, q3
            roll, pitch, yaw,
        )

    def sync_viewer(self):
        if self.viewer is not None:
            with self._viewer_lock:
                self.viewer.sync()


def main():
    print(f"[mujoco_node] Loading model: {MODEL_PATH}")
    sim = MuJoCoSim(MODEL_PATH)
    print(f"[mujoco_node] Model loaded (timestep={sim.model.opt.timestep}s)")

    # Optional viewer
    if os.environ.get("MUJOCO_VIEWER", "0") == "1":
        sim.launch_viewer()

    node = Node("mujoco_node")

    # Track viewer sync rate
    last_viewer_sync = 0.0
    VIEWER_SYNC_INTERVAL = 1.0 / 60.0  # 60 Hz max

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "motor_commands":
                raw = bytes(event["value"].to_pylist())
                num_axes = len(raw) // AXIS_REF_SIZE

                commands = []
                for i in range(min(num_axes, NUM_AXES)):
                    cmd = struct.unpack_from(AXIS_REF_FMT, raw, i * AXIS_REF_SIZE)
                    commands.append(cmd)

                # Apply commands; skip dynamics while frozen (pre-RUN)
                should_step = sim.apply_commands(commands)
                if should_step:
                    sim.step()
                else:
                    # Update sensors without stepping dynamics
                    sim.forward()

                # Send motor_status
                motor_bytes = sim.get_motor_status()
                node.send_output(
                    "motor_status",
                    pa.array(list(motor_bytes), type=pa.uint8()),
                )

                # Send imu_data
                imu_bytes = sim.get_imu_data()
                node.send_output(
                    "imu_data",
                    pa.array(list(imu_bytes), type=pa.uint8()),
                )

                # Sync viewer at limited rate
                if sim.viewer is not None:
                    now = time.monotonic()
                    if now - last_viewer_sync >= VIEWER_SYNC_INTERVAL:
                        sim.sync_viewer()
                        last_viewer_sync = now

    print("[mujoco_node] Shutting down")


if __name__ == "__main__":
    main()
