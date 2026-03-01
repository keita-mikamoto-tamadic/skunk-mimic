from dora import Node
import struct
import time
from rich.live import Live
from rich.table import Table
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
# sizeof(AxisAct) = 32
AXIS_ACT_FMT = "<dddB7x"  # little-endian
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32

# State enum with styles (C++ enum class State と対応)
STATE_STYLES = {
    0: ("OFF", "dim white"),
    1: ("STOP", "yellow bold"),
    2: ("READY", "blue bold"),
    3: ("RUN", "green bold")
}


def build_table(axes, timestamp_ns=None, state=None, config=None):
    # Title: ロボット名を使用
    title = config.robot_name if config else "motor_status"
    table = Table(show_header=True, title=title)
    table.add_column("axis", justify="left", style="cyan")
    table.add_column("CAN", justify="right", style="dim")
    table.add_column("position", justify="right")
    table.add_column("velocity", justify="right")
    table.add_column("torque", justify="right")
    table.add_column("fault", justify="center")

    for i, (pos, vel, torq, fault) in enumerate(axes):
        # 軸名と CAN ID を config から取得
        axis_name = config.axes[i].name if config and i < len(config.axes) else f"#{i}"
        can_id = str(config.axes[i].device_id) if config and i < len(config.axes) else "-"

        fault_style = "red bold" if fault != 0 else "green"
        table.add_row(
            axis_name,
            can_id,
            f"{pos:+.4f}",
            f"{vel:+.4f}",
            f"{torq:+.4f}",
            f"[{fault_style}]{fault}[/]",
        )

    # State と Time を一番下に追加
    if state is not None or timestamp_ns is not None:
        table.add_section()
        info_parts = []
        if state is not None:
            state_name, state_style = STATE_STYLES.get(state, ("?", "red"))
            info_parts.append(f"State: [{state_style}]{state_name}[/]")
        if timestamp_ns is not None:
            timestamp_sec = timestamp_ns / 1_000_000_000
            info_parts.append(f"Time: [dim]{timestamp_sec:.3f}s[/]")

        info_text = "  |  ".join(info_parts)
        table.add_row(info_text, "", "", "", "", "")

    return table


node = Node("data_viewer")

# Load robot configuration
config = robot_config.load_from_file(CONFIG_PATH)
print(f"Loaded config: {config.robot_name} ({config.axis_count} axes)")

with Live(build_table([], config=config), refresh_per_second=30) as live:
    start_time = time.time_ns()
    current_state = None
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "state_status":
                raw = bytes(event["value"].to_pylist())
                if len(raw) > 0:
                    current_state = raw[0]
            elif event["id"] == "motor_status":
                raw = bytes(event["value"].to_pylist())
                axis_count = len(raw) // AXIS_ACT_SIZE

                # Get current timestamp relative to start
                timestamp_ns = time.time_ns() - start_time

                axes = []
                for i in range(axis_count):
                    axes.append(
                        struct.unpack_from(AXIS_ACT_FMT, raw, i * AXIS_ACT_SIZE)
                    )
                live.update(build_table(axes, timestamp_ns, current_state, config))
