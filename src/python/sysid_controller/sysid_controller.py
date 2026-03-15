"""SysID controller for the new RCM passthrough architecture.

Sends state_command to drive RCM through OFF→STOP→READY→RUN,
then sends run_command (AxisRef[]) with excitation patterns during RUN.

Waits for recorder_ready from data_recorder before starting.
Monitors motor_status for faults.

Usage:
  uv run sysid_controller/sysid_controller.py [--axes wheel] [--pattern step+chirp]
                                                [--startup-delay 3]
"""

import argparse
import math
import os
import struct
import sys
import time

import pyarrow as pa
from dora import Node

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
# Enums (must match C++ enum_def.hpp)
# ---------------------------------------------------------------------------
# MotorState
MOTOR_OFF = 0
MOTOR_STOP = 1
MOTOR_POSITION = 2
MOTOR_VELOCITY = 3

# StateCommand
CMD_STOP = 0
CMD_RUN = 1
CMD_SERVO_OFF = 2
CMD_SERVO_ON = 3
CMD_READY = 5

# State
STATE_OFF = 0
STATE_STOP = 1
STATE_READY = 2
STATE_RUN = 3

NUM_AXES = 6
WHEEL_R = 2
WHEEL_L = 5

# ---------------------------------------------------------------------------
# Excitation patterns (same as sysid_input.py)
# ---------------------------------------------------------------------------

class StepPattern:
    def __init__(self, amplitude=5.0, period=3.0, num_cycles=3):
        self.amplitude = amplitude
        self.period = period
        self.num_cycles = num_cycles
        self.total_duration = period * num_cycles * 2

    def value(self, t: float) -> float:
        if t >= self.total_duration:
            return 0.0
        cycle_pos = (t % (self.period * 2)) / (self.period * 2)
        return self.amplitude if cycle_pos < 0.5 else -self.amplitude


class ChirpPattern:
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


def build_pattern(pattern_str: str):
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


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pack_axis_ref(motor_state, ref_val, kp_scale, kv_scale,
                  velocity_limit, accel_limit, torque_limit):
    return struct.pack(
        AXIS_REF_FMT,
        motor_state, ref_val, kp_scale, kv_scale,
        velocity_limit, accel_limit, torque_limit,
    )


def send_state_command(node, cmd):
    node.send_output("state_command", pa.array([cmd], type=pa.uint8()))


def send_run_command(node, buf):
    node.send_output("run_command", pa.array(list(bytes(buf)), type=pa.uint8()))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="SysID controller (RCM passthrough)")
    parser.add_argument("--axes", default="wheel", help="Target axes: 'wheel'")
    parser.add_argument("--pattern", default="step+chirp", help="step, chirp, step+chirp")
    parser.add_argument("--startup-delay", type=float, default=3.0,
                        help="Seconds to wait in RUN before excitation starts")
    return parser.parse_args()


def main():
    args = parse_args()

    config = robot_config.load_from_file(CONFIG_PATH)
    print(f"[sysid_ctrl] Config: {config.robot_name} ({config.axis_count} axes)")

    pattern = build_pattern(args.pattern)
    print(f"[sysid_ctrl] Pattern: {args.pattern} ({pattern.total_duration:.1f}s)")

    if args.axes == "wheel":
        target_axes = [WHEEL_R, WHEEL_L]
    else:
        raise ValueError(f"Unknown axis group: {args.axes}")
    print(f"[sysid_ctrl] Target axes: {[config.axes[i].name for i in target_axes]}")

    node = Node("sysid_controller")

    # Wait for recorder_ready before driving state machine
    recorder_ready = False
    current_state = STATE_OFF
    sent_servo_on = False
    sent_ready = False
    sent_run = False
    excitation_start = None
    run_start = None
    finished = False
    tick_count = 0

    for event in node:
        if event["type"] == "STOP":
            print("[sysid_ctrl] STOP event -> sending SERVO_OFF")
            send_state_command(node, CMD_SERVO_OFF)
            break

        if event["type"] != "INPUT":
            continue

        eid = event["id"]

        if eid == "recorder_ready":
            recorder_ready = True
            print("[sysid_ctrl] Recorder ready, starting state machine")

        elif eid == "state_status":
            raw = bytes(event["value"].to_pylist())
            if len(raw) >= 1:
                current_state = raw[0]

        elif eid == "motor_status":
            # motor_status 駆動: fault 監視 + 制御計算
            raw = bytes(event["value"].to_pylist())
            for i in range(min(len(raw) // AXIS_ACT_SIZE, NUM_AXES)):
                _, _, _, fault = struct.unpack_from(AXIS_ACT_FMT, raw, i * AXIS_ACT_SIZE)
                if fault != 0:
                    print(f"[sysid_ctrl] FAULT on axis {i}: {fault}")

            tick_count += 1
            now = time.monotonic()

            if not recorder_ready:
                if tick_count % 333 == 0:
                    print("[sysid_ctrl] Waiting for data_recorder...")
                continue

            # Drive state machine forward
            if current_state == STATE_OFF and not sent_servo_on:
                send_state_command(node, CMD_SERVO_ON)
                sent_servo_on = True
                print("[sysid_ctrl] Sent SERVO_ON")

            elif current_state == STATE_STOP and not sent_ready:
                send_state_command(node, CMD_READY)
                sent_ready = True
                print("[sysid_ctrl] Sent READY")

            elif current_state == STATE_READY and not sent_run:
                # READY might still be interpolating, RCM will reject RUN until done
                send_state_command(node, CMD_RUN)
                # Don't set sent_run here; retry until RCM accepts

            elif current_state == STATE_RUN:
                if not sent_run:
                    sent_run = True
                    run_start = now
                    print("[sysid_ctrl] RUN state entered")

                elapsed = now - run_start

                # Build run_command
                cmd_buf = bytearray()
                for i in range(NUM_AXES):
                    ax = config.axes[i]

                    if i not in target_axes:
                        # Non-target: POSITION hold at initial pose
                        cmd_buf += pack_axis_ref(
                            MOTOR_POSITION, ax.initial_position,
                            1.0, 1.0,
                            float('nan'), float('nan'), ax.torque_limit,
                        )
                    elif elapsed < args.startup_delay:
                        # Startup delay: wheels hold zero velocity
                        cmd_buf += pack_axis_ref(
                            MOTOR_VELOCITY, 0.0,
                            0.0, 20.0,
                            float('nan'), float('nan'), ax.torque_limit,
                        )
                    elif not finished:
                        # Excitation
                        if excitation_start is None:
                            excitation_start = now
                            print("[sysid_ctrl] Starting excitation")

                        t = now - excitation_start
                        if t >= pattern.total_duration:
                            if not finished:
                                print("[sysid_ctrl] Excitation complete -> STOP")
                                finished = True
                            cmd_buf += pack_axis_ref(
                                MOTOR_VELOCITY, 0.0,
                                0.0, 20.0,
                                float('nan'), float('nan'), ax.torque_limit,
                            )
                        else:
                            vel_cmd = pattern.value(t)
                            vel_cmd = max(-30.0, min(30.0, vel_cmd))
                            cmd_buf += pack_axis_ref(
                                MOTOR_VELOCITY, vel_cmd,
                                0.0, 20.0,
                                float('nan'), float('nan'), ax.torque_limit,
                            )
                    else:
                        # Post-excitation: zero velocity
                        cmd_buf += pack_axis_ref(
                            MOTOR_VELOCITY, 0.0,
                            0.0, 20.0,
                            float('nan'), float('nan'), ax.torque_limit,
                        )

                send_run_command(node, cmd_buf)

                if finished:
                    send_state_command(node, CMD_STOP)

                # Progress
                if tick_count % 333 == 0:
                    if excitation_start is None:
                        remaining = args.startup_delay - elapsed
                        print(f"[sysid_ctrl] Startup delay ({remaining:.1f}s remaining)")
                    elif not finished:
                        t = now - excitation_start
                        print(f"[sysid_ctrl] t={t:.1f}/{pattern.total_duration:.1f}s")

    print("[sysid_ctrl] Shutting down")


if __name__ == "__main__":
    main()
