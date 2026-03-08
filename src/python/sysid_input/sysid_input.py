"""Excitation signal generator for system identification.

Combines dummy_input (state commands) and robot_control_manager (motor commands)
into a single node. Directly sends AxisRef[6] motor_commands.

Non-target axes are held at initial_position via POSITION mode.
Target axes (wheels) receive step or chirp excitation in VELOCITY mode.

Usage:
  uv run sysid_input/sysid_input.py [--axes wheel] [--pattern step+chirp]
                                     [--duration 60] [--startup-delay 3]
"""

import argparse
import math
import os
import struct
import sys
import time

import numpy as np
import pyarrow as pa
from dora import Node

# lib を import パスに追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib import robot_config

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "robot_config", "mimic_v2.json")

# ---------------------------------------------------------------------------
# Binary formats (must match C++ structs)
# ---------------------------------------------------------------------------
AXIS_REF_FMT = "<B7xdddddd"
AXIS_REF_SIZE = struct.calcsize(AXIS_REF_FMT)  # 56

AXIS_ACT_FMT = "<dddB7x"
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32

# ---------------------------------------------------------------------------
# MotorState enum
# ---------------------------------------------------------------------------
MOTOR_OFF = 0
MOTOR_STOP = 1
MOTOR_POSITION = 2
MOTOR_VELOCITY = 3

NUM_AXES = 6

# Axis indices
WHEEL_R = 2
WHEEL_L = 5
HIP_R = 0
KNEE_R = 1
HIP_L = 3
KNEE_L = 4

# Base servo gains (must match mujoco_node.py / moteus config)
BASE_KP = 50.0
BASE_KV = 20.0

# ---------------------------------------------------------------------------
# Excitation patterns
# ---------------------------------------------------------------------------

class StepPattern:
    """Step response: alternating ±amplitude with hold period."""

    def __init__(self, amplitude=5.0, period=3.0, num_cycles=3):
        self.amplitude = amplitude
        self.period = period
        self.num_cycles = num_cycles
        self.total_duration = period * num_cycles * 2  # pos + neg per cycle

    def value(self, t: float) -> float:
        if t >= self.total_duration:
            return 0.0
        cycle_pos = (t % (self.period * 2)) / (self.period * 2)
        return self.amplitude if cycle_pos < 0.5 else -self.amplitude


class ChirpPattern:
    """Linear chirp: frequency sweeps from f0 to f1."""

    def __init__(self, f0=0.5, f1=10.0, amplitude=3.0, duration=20.0):
        self.f0 = f0
        self.f1 = f1
        self.amplitude = amplitude
        self.duration = duration
        self.total_duration = duration

    def value(self, t: float) -> float:
        if t >= self.duration:
            return 0.0
        # Linear chirp: instantaneous freq = f0 + (f1-f0)*t/T
        phase = 2 * math.pi * (self.f0 * t + 0.5 * (self.f1 - self.f0) * t * t / self.duration)
        return self.amplitude * math.sin(phase)


class CombinedPattern:
    """Runs multiple patterns in sequence."""

    def __init__(self, patterns):
        self.patterns = patterns
        self.offsets = []
        offset = 0.0
        for p in patterns:
            self.offsets.append(offset)
            offset += p.total_duration
        self.total_duration = offset

    def value(self, t: float) -> float:
        for i, p in enumerate(self.patterns):
            if t < self.offsets[i] + p.total_duration:
                return p.value(t - self.offsets[i])
        return 0.0


def build_pattern(pattern_str: str) -> object:
    """Build excitation pattern from string specification."""
    patterns = []
    for name in pattern_str.split("+"):
        name = name.strip().lower()
        if name == "step":
            patterns.append(StepPattern())
        elif name == "chirp":
            patterns.append(ChirpPattern())
        else:
            raise ValueError(f"Unknown pattern: {name}")

    if len(patterns) == 1:
        return patterns[0]
    return CombinedPattern(patterns)


def pack_axis_ref(motor_state, ref_val, kp_scale, kv_scale,
                  velocity_limit, accel_limit, torque_limit):
    """Pack a single AxisRef struct."""
    return struct.pack(
        AXIS_REF_FMT,
        motor_state,
        ref_val,
        kp_scale,
        kv_scale,
        velocity_limit,
        accel_limit,
        torque_limit,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="SysID excitation signal generator")
    parser.add_argument("--axes", default="wheel",
                        help="Target axes: 'wheel' (default)")
    parser.add_argument("--pattern", default="step+chirp",
                        help="Excitation pattern: step, chirp, step+chirp (default)")
    parser.add_argument("--startup-delay", type=float, default=3.0,
                        help="Seconds to hold initial position before excitation")
    return parser.parse_args()


def main():
    args = parse_args()

    # Load robot config
    config = robot_config.load_from_file(CONFIG_PATH)
    print(f"[sysid_input] Config: {config.robot_name} ({config.axis_count} axes)")

    # Build excitation pattern
    pattern = build_pattern(args.pattern)
    print(f"[sysid_input] Pattern: {args.pattern} (duration: {pattern.total_duration:.1f}s)")

    # Determine target axes
    if args.axes == "wheel":
        target_axes = [WHEEL_R, WHEEL_L]
    else:
        raise ValueError(f"Unknown axis group: {args.axes}")
    print(f"[sysid_input] Target axes: {[config.axes[i].name for i in target_axes]}")

    node = Node("sysid_input")

    # State tracking
    excitation_start = None
    startup_complete = False
    finished = False
    tick_count = 0
    node_start = time.monotonic()

    for event in node:
        if event["type"] == "INPUT":
            eid = event["id"]

            if eid == "tick":
                tick_count += 1
                now = time.monotonic()
                elapsed = now - node_start

                # Build AxisRef[6]
                cmd_buf = bytearray()

                for i in range(NUM_AXES):
                    ax = config.axes[i]

                    if i in target_axes:
                        # Target axis: excitation signal
                        if elapsed < args.startup_delay:
                            # Startup phase: hold position
                            cmd_buf += pack_axis_ref(
                                MOTOR_POSITION,
                                ax.initial_position,
                                1.0, 1.0,
                                float('nan'), float('nan'),
                                ax.torque_limit,
                            )
                        elif not finished:
                            if excitation_start is None:
                                excitation_start = now
                                print("[sysid_input] Starting excitation")

                            t = now - excitation_start
                            if t >= pattern.total_duration:
                                # Excitation complete: stop wheel
                                if not finished:
                                    print("[sysid_input] Excitation complete")
                                    finished = True
                                cmd_buf += pack_axis_ref(
                                    MOTOR_VELOCITY, 0.0,
                                    0.0, 1.0,
                                    float('nan'), float('nan'),
                                    ax.torque_limit,
                                )
                            else:
                                # Active excitation
                                vel_cmd = pattern.value(t)
                                # Safety clamp
                                max_vel = 30.0
                                vel_cmd = max(-max_vel, min(max_vel, vel_cmd))
                                cmd_buf += pack_axis_ref(
                                    MOTOR_VELOCITY,
                                    vel_cmd,
                                    0.0, 1.0,
                                    float('nan'), float('nan'),
                                    ax.torque_limit,
                                )
                        else:
                            # Post-excitation: velocity zero (brake)
                            cmd_buf += pack_axis_ref(
                                MOTOR_VELOCITY, 0.0,
                                0.0, 1.0,
                                float('nan'), float('nan'),
                                ax.torque_limit,
                            )
                    else:
                        # Non-target axis: hold initial position
                        cmd_buf += pack_axis_ref(
                            MOTOR_POSITION,
                            ax.initial_position,
                            1.0, 1.0,
                            float('nan') if math.isnan(ax.velocity_limit) else ax.velocity_limit,
                            float('nan') if math.isnan(ax.accel_limit) else ax.accel_limit,
                            ax.torque_limit,
                        )

                # Send motor_commands
                node.send_output(
                    "motor_commands",
                    pa.array(list(bytes(cmd_buf)), type=pa.uint8()),
                )

                # Progress report
                if tick_count % 333 == 0:  # ~1Hz at 333Hz tick
                    if excitation_start is not None and not finished:
                        t = now - excitation_start
                        print(f"[sysid_input] t={t:.1f}/{pattern.total_duration:.1f}s")

            elif eid == "motor_status":
                # Could monitor faults here if needed
                pass

    print("[sysid_input] Shutting down")


if __name__ == "__main__":
    main()
