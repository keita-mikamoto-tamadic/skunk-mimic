"""Convert recorded .npz data to mujoco.sysid.ModelSequences.

Reads npz files from sysid/data/ and constructs ModelSequences objects
using TimeSeries.from_control_names / from_names for automatic signal mapping.

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

# Sensor names in mimic_v2_sysid.xml (joint sensors only, matching data columns)
JPOS_SENSOR_NAMES = [
    "jpos_hip_pitch_r", "jpos_knee_r", "jpos_wheel_r",
    "jpos_hip_pitch_l", "jpos_knee_l", "jpos_wheel_l",
]
JVEL_SENSOR_NAMES = [
    "jvel_hip_pitch_r", "jvel_knee_r", "jvel_wheel_r",
    "jvel_hip_pitch_l", "jvel_knee_l", "jvel_wheel_l",
]
SENSOR_NAMES = JPOS_SENSOR_NAMES + JVEL_SENSOR_NAMES

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
    """Trim data to excitation interval only (where wheels are in VELOCITY mode)."""
    states = data["cmd_motor_state"]
    wheel_active = (states[:, WHEEL_R] == MOTOR_VELOCITY) | \
                   (states[:, WHEEL_L] == MOTOR_VELOCITY)
    active_indices = np.where(wheel_active)[0]

    if len(active_indices) == 0:
        raise ValueError("No excitation found (no VELOCITY commands on wheels)")

    cmd_start = active_indices[0]
    cmd_end = active_indices[-1] + 1

    t_start = data["cmd_timestamps"][cmd_start]
    t_end = data["cmd_timestamps"][cmd_end - 1]
    print(f"  Excitation window: {t_start:.2f}s - {t_end:.2f}s "
          f"({cmd_end - cmd_start} cmd samples)")

    trimmed = {}
    for key in data:
        arr = data[key]
        if key.startswith("cmd_"):
            trimmed[key] = arr[cmd_start:cmd_end]
        elif key.startswith("status_"):
            st = data["status_timestamps"]
            mask = (st >= t_start) & (st <= t_end)
            trimmed[key] = arr[mask]
        elif key.startswith("imu_"):
            it = data["imu_timestamps"]
            mask = (it >= t_start) & (it <= t_end)
            trimmed[key] = arr[mask]
        else:
            trimmed[key] = arr

    t0 = trimmed["cmd_timestamps"][0]
    trimmed["cmd_timestamps"] = trimmed["cmd_timestamps"] - t0
    if "status_timestamps" in trimmed:
        trimmed["status_timestamps"] = trimmed["status_timestamps"] - t0
    if "imu_timestamps" in trimmed:
        trimmed["imu_timestamps"] = trimmed["imu_timestamps"] - t0

    n_status = len(trimmed.get("status_timestamps", []))
    print(f"  Trimmed: {cmd_end - cmd_start} cmd, {n_status} status samples")
    return trimmed


def build_model_sequences(npz_path: str) -> sysid.ModelSequences:
    """Convert npz recording to ModelSequences."""
    print(f"Loading {npz_path}")
    data = load_npz(npz_path)

    n_cmd = len(data["cmd_timestamps"])
    n_status = len(data["status_timestamps"])
    print(f"  Raw: {n_cmd} cmd, {n_status} status, "
          f"duration: {data['status_timestamps'][-1]:.1f}s")

    data = trim_to_excitation(data)

    spec = mujoco.MjSpec.from_file(MODEL_PATH)
    model = spec.compile()

    # Control: ref_val for all 6 actuators
    control = sysid.TimeSeries.from_control_names(
        times=data["cmd_timestamps"],
        data=data["cmd_ref_val"],
        model=model,
        names=ACTUATOR_NAMES,
    )

    # Sensor: joint positions (6) + joint velocities (6) = 12 columns
    sensor_data = np.hstack([
        data["status_position"],
        data["status_velocity"],
    ])
    sensordata = sysid.TimeSeries.from_names(
        times=data["status_timestamps"],
        data=sensor_data,
        model=model,
        names=SENSOR_NAMES,
    )

    initial_state = sysid.create_initial_state(
        model=model,
        qpos=data["status_position"][0],
        qvel=data["status_velocity"][0],
    )

    basename = os.path.splitext(os.path.basename(npz_path))[0]
    return sysid.ModelSequences(
        name=basename,
        spec=spec,
        sequence_name=basename,
        initial_state=initial_state,
        control=control,
        sensordata=sensordata,
    )


def main():
    parser = argparse.ArgumentParser(description="Convert npz recording to sysid format")
    parser.add_argument("npz_file", help="Path to .npz recording")
    args = parser.parse_args()

    ms = build_model_sequences(args.npz_file)
    print(f"ModelSequences created: {ms.name}")


if __name__ == "__main__":
    main()
