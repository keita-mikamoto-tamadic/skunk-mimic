"""Convert recorded .npz data to mujoco.sysid.ModelSequences.

Reads npz files from sysid/data/ and constructs ModelSequences objects
with control (ref_val[6]) and sensordata (jointpos[6] + jointvel[6]) TimeSeries.

Usage:
    uv run sysid/convert_recording.py <npz_file>
"""

import argparse
import os
import sys

import mujoco
import numpy as np
from mujoco import sysid

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
MODEL_PATH = os.path.join(PROJECT_ROOT, "sim", "mimic_v2_sysid.xml")

# Sensor names in mimic_v2_sysid.xml (must match order)
JPOS_SENSOR_NAMES = [
    "jpos_hip_pitch_r", "jpos_knee_r", "jpos_wheel_r",
    "jpos_hip_pitch_l", "jpos_knee_l", "jpos_wheel_l",
]
JVEL_SENSOR_NAMES = [
    "jvel_hip_pitch_r", "jvel_knee_r", "jvel_wheel_r",
    "jvel_hip_pitch_l", "jvel_knee_l", "jvel_wheel_l",
]

# Actuator names in the model
ACTUATOR_NAMES = [
    "hip_pitch_r", "knee_r", "wheel_r",
    "hip_pitch_l", "knee_l", "wheel_l",
]


MOTOR_VELOCITY = 3

# Wheel axis indices
WHEEL_R = 2
WHEEL_L = 5


def load_npz(filepath: str) -> dict:
    """Load and validate npz recording."""
    data = dict(np.load(filepath))
    required = ["cmd_timestamps", "cmd_ref_val", "cmd_motor_state",
                 "status_timestamps", "status_position", "status_velocity"]
    for key in required:
        if key not in data:
            raise ValueError(f"Missing key '{key}' in {filepath}")
    return data


def trim_to_excitation(data: dict) -> dict:
    """Trim data to excitation interval only (where wheels are in VELOCITY mode).

    Removes STOP periods before/after excitation so that sysid rollout
    only sees the active region where the actuator model matches.
    """
    states = data["cmd_motor_state"]
    # Wheel is active when either wheel is in VELOCITY mode
    wheel_active = (states[:, WHEEL_R] == MOTOR_VELOCITY) | \
                   (states[:, WHEEL_L] == MOTOR_VELOCITY)
    active_indices = np.where(wheel_active)[0]

    if len(active_indices) == 0:
        raise ValueError("No excitation found (no VELOCITY commands on wheels)")

    # Contiguous range from first to last active command
    cmd_start = active_indices[0]
    cmd_end = active_indices[-1] + 1

    t_start = data["cmd_timestamps"][cmd_start]
    t_end = data["cmd_timestamps"][cmd_end - 1]
    print(f"  Excitation window: {t_start:.2f}s - {t_end:.2f}s "
          f"({cmd_end - cmd_start} cmd samples)")

    # Trim command arrays
    trimmed = {}
    for key in data:
        arr = data[key]
        if key.startswith("cmd_"):
            trimmed[key] = arr[cmd_start:cmd_end]
        elif key.startswith("status_"):
            # Trim status to matching time window
            st = data["status_timestamps"]
            mask = (st >= t_start) & (st <= t_end)
            trimmed[key] = arr[mask]
        elif key.startswith("imu_"):
            it = data["imu_timestamps"]
            mask = (it >= t_start) & (it <= t_end)
            trimmed[key] = arr[mask]
        else:
            trimmed[key] = arr

    # Re-zero timestamps
    t0 = trimmed["cmd_timestamps"][0]
    trimmed["cmd_timestamps"] = trimmed["cmd_timestamps"] - t0
    if "status_timestamps" in trimmed:
        trimmed["status_timestamps"] = trimmed["status_timestamps"] - t0
    if "imu_timestamps" in trimmed:
        trimmed["imu_timestamps"] = trimmed["imu_timestamps"] - t0

    n_status = len(trimmed.get("status_timestamps", []))
    print(f"  Trimmed: {cmd_end - cmd_start} cmd, {n_status} status samples")
    return trimmed


def build_control_timeseries(data: dict) -> sysid.TimeSeries:
    """Build control TimeSeries from command data."""
    times = data["cmd_timestamps"]
    ctrl = data["cmd_ref_val"]  # (N, 6)

    # signal_mapping: name → (SignalType, column_indices)
    signal_mapping = {
        name: (sysid.SignalType.MjCtrl, np.array([i]))
        for i, name in enumerate(ACTUATOR_NAMES)
    }

    return sysid.TimeSeries(
        times=times,
        data=ctrl,
        signal_mapping=signal_mapping,
    )


def build_sensor_timeseries(data: dict) -> sysid.TimeSeries:
    """Build sensordata TimeSeries from status data.

    Combines joint positions (6) and joint velocities (6) = 12 channels.
    """
    times = data["status_timestamps"]
    positions = data["status_position"]   # (N, 6)
    velocities = data["status_velocity"]  # (N, 6)
    sensor_data = np.hstack([positions, velocities])  # (N, 12)

    # signal_mapping: name → (SignalType, column_indices)
    signal_mapping = {}
    for i, name in enumerate(JPOS_SENSOR_NAMES):
        signal_mapping[name] = (sysid.SignalType.MjSensor, np.array([i]))
    for i, name in enumerate(JVEL_SENSOR_NAMES):
        signal_mapping[name] = (sysid.SignalType.MjSensor, np.array([6 + i]))

    return sysid.TimeSeries(
        times=times,
        data=sensor_data,
        signal_mapping=signal_mapping,
    )


def build_initial_state(data: dict, model: mujoco.MjModel) -> np.ndarray:
    """Build initial state from first status frame."""
    qpos = data["status_position"][0]
    qvel = data["status_velocity"][0]
    return sysid.create_initial_state(model=model, qpos=qpos, qvel=qvel)


def build_model_sequences(npz_path: str) -> sysid.ModelSequences:
    """Convert npz recording to ModelSequences (in-memory, no pickle)."""
    print(f"Loading {npz_path}")
    data = load_npz(npz_path)

    n_cmd = len(data["cmd_timestamps"])
    n_status = len(data["status_timestamps"])
    print(f"  Raw: {n_cmd} cmd, {n_status} status, "
          f"duration: {data['status_timestamps'][-1]:.1f}s")

    # Trim to excitation-only interval
    data = trim_to_excitation(data)

    spec = mujoco.MjSpec.from_file(MODEL_PATH)
    model = spec.compile()

    control = build_control_timeseries(data)
    sensordata = build_sensor_timeseries(data)
    initial_state = build_initial_state(data, model)

    basename = os.path.splitext(os.path.basename(npz_path))[0]
    return sysid.ModelSequences(
        name=basename,
        spec=spec,
        sequence_name=basename,
        initial_state=initial_state,
        control=control,
        sensordata=sensordata,
        allow_missing_sensors=True,
    )


def main():
    parser = argparse.ArgumentParser(description="Convert npz recording to sysid format")
    parser.add_argument("npz_file", help="Path to .npz recording")
    args = parser.parse_args()

    ms = build_model_sequences(args.npz_file)
    print(f"ModelSequences created: {ms.name}")


if __name__ == "__main__":
    main()
