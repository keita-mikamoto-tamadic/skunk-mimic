"""Excitation signal generator for system identification.

Bypasses robot_control_manager and sends motor_commands directly.

State sequence:
  1. STOP: All axes hold current position (servo on, brake)
  2. Wait for --startup-delay seconds
  3. Excitation: target axes (wheels) receive step/chirp in VELOCITY mode,
     non-target axes (hip/knee) remain in STOP
  4. Post-excitation: all axes STOP

Usage:
  uv run sysid_input/sysid_input.py [--axes wheel] [--pattern step+chirp]
                                     [--startup-delay 3]
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

# ---------------------------------------------------------------------------
# Excitation patterns
# ---------------------------------------------------------------------------

class StepPattern:
    """Step response: alternating +/- amplitude with hold period."""

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


def pack_stop(torque_limit):
    """Pack AxisRef for STOP mode (hold current position, brake)."""
    return pack_axis_ref(
        MOTOR_STOP,
        0.0,             # ref_val unused in STOP (moteus uses NaN position)
        1.0, 1.0,
        float('nan'), float('nan'),
        torque_limit,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="SysID excitation signal generator")
    parser.add_argument("--axes", default="wheel",
                        help="Target axes: 'wheel' (default)")
    parser.add_argument("--pattern", default="step+chirp",
                        help="Excitation pattern: step, chirp, step+chirp (default)")
    parser.add_argument("--startup-delay", type=float, default=3.0,
                        help="Seconds in STOP before excitation starts")
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
    print(f"[sysid_input] All axes start in STOP (hold current position)")
    print(f"[sysid_input] Excitation begins after {args.startup_delay}s delay")

    node = Node("sysid_input")

    def send_all_off(node, config):
        """Send MOTOR_OFF to all axes."""
        cmd_buf = bytearray()
        for i in range(NUM_AXES):
            cmd_buf += pack_axis_ref(
                MOTOR_OFF, 0.0, 0.0, 0.0,
                float('nan'), float('nan'),
                config.axes[i].torque_limit,
            )
        node.send_output(
            "motor_commands",
            pa.array(list(bytes(cmd_buf)), type=pa.uint8()),
        )

    # State tracking
    recorder_ready = False
    excitation_start = None
    ready_time = None      # monotonic time when recorder_ready received
    finished = False
    tick_count = 0

    for event in node:
        if event["type"] == "STOP":
            # Dora shutdown: servo off all axes
            print("[sysid_input] STOP event -> sending MOTOR_OFF to all axes")
            send_all_off(node, config)
            break

        if event["type"] == "INPUT":
            eid = event["id"]

            if eid == "recorder_ready":
                recorder_ready = True
                ready_time = time.monotonic()
                print("[sysid_input] Recorder ready, starting startup delay")

            elif eid == "tick":
                tick_count += 1
                now = time.monotonic()
                elapsed = (now - ready_time) if ready_time else 0.0

                cmd_buf = bytearray()

                if not recorder_ready:
                    # Waiting for recorder: all axes STOP
                    for i in range(NUM_AXES):
                        cmd_buf += pack_stop(config.axes[i].torque_limit)
                    node.send_output(
                        "motor_commands",
                        pa.array(list(bytes(cmd_buf)), type=pa.uint8()),
                    )
                    if tick_count % 333 == 0:
                        print("[sysid_input] Waiting for data_recorder...")
                    continue

                for i in range(NUM_AXES):
                    ax = config.axes[i]

                    if i not in target_axes:
                        # Non-target axes: always STOP (hold current position)
                        cmd_buf += pack_stop(ax.torque_limit)

                    elif elapsed < args.startup_delay:
                        # Target axes during startup: STOP
                        cmd_buf += pack_stop(ax.torque_limit)

                    elif not finished:
                        # Target axes: active excitation
                        if excitation_start is None:
                            excitation_start = now
                            print("[sysid_input] Starting excitation")

                        t = now - excitation_start
                        if t >= pattern.total_duration:
                            print("[sysid_input] Excitation complete")
                            finished = True
                            cmd_buf += pack_stop(ax.torque_limit)
                        else:
                            vel_cmd = pattern.value(t)
                            # Safety clamp
                            max_vel = 30.0
                            vel_cmd = max(-max_vel, min(max_vel, vel_cmd))
                            cmd_buf += pack_axis_ref(
                                MOTOR_VELOCITY,
                                vel_cmd,
                                0.0, 20.0,
                                float('nan'), float('nan'),
                                ax.torque_limit,
                            )
                    else:
                        # Post-excitation: STOP
                        cmd_buf += pack_stop(ax.torque_limit)

                # Send motor_commands
                node.send_output(
                    "motor_commands",
                    pa.array(list(bytes(cmd_buf)), type=pa.uint8()),
                )

                # Progress report
                if tick_count % 333 == 0:  # ~1Hz at 333Hz tick
                    if excitation_start is None:
                        remaining = args.startup_delay - elapsed
                        print(f"[sysid_input] STOP (excitation in {remaining:.1f}s)")
                    elif not finished:
                        t = now - excitation_start
                        print(f"[sysid_input] t={t:.1f}/{pattern.total_duration:.1f}s")

            elif eid == "motor_status":
                # Monitor for faults
                raw = bytes(event["value"].to_pylist())
                for i in range(min(len(raw) // AXIS_ACT_SIZE, NUM_AXES)):
                    _, _, _, fault = struct.unpack_from(AXIS_ACT_FMT, raw, i * AXIS_ACT_SIZE)
                    if fault != 0:
                        print(f"[sysid_input] FAULT on axis {i}: {fault}")

    print("[sysid_input] Shutting down")


if __name__ == "__main__":
    main()
