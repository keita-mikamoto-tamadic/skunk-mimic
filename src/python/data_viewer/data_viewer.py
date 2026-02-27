from dora import Node
import struct
import time
from rich.live import Live
from rich.table import Table

# AxisAct: double position, double velocity, double torque, uint8_t fault (+7 padding)
# sizeof(AxisAct) = 32
AXIS_ACT_FMT = "<dddB7x"  # little-endian
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32


def build_table(axes, timestamp_ns=None):
    title = "motor_status"
    if timestamp_ns is not None:
        # Convert nanoseconds to seconds with millisecond precision
        timestamp_sec = timestamp_ns / 1_000_000_000
        title = f"motor_status @ {timestamp_sec:.3f}s"

    table = Table(title=title)
    table.add_column("#", justify="right", style="cyan")
    table.add_column("position", justify="right")
    table.add_column("velocity", justify="right")
    table.add_column("torque", justify="right")
    table.add_column("fault", justify="center")

    for i, (pos, vel, torq, fault) in enumerate(axes):
        fault_style = "red bold" if fault != 0 else "green"
        table.add_row(
            str(i),
            f"{pos:+.4f}",
            f"{vel:+.4f}",
            f"{torq:+.4f}",
            f"[{fault_style}]{fault}[/]",
        )
    return table


node = Node("data_viewer")

with Live(build_table([]), refresh_per_second=30) as live:
    start_time = time.time_ns()
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "motor_status":
            raw = bytes(event["value"].to_pylist())
            axis_count = len(raw) // AXIS_ACT_SIZE

            # Get current timestamp relative to start
            timestamp_ns = time.time_ns() - start_time

            axes = []
            for i in range(axis_count):
                axes.append(
                    struct.unpack_from(AXIS_ACT_FMT, raw, i * AXIS_ACT_SIZE)
                )
            live.update(build_table(axes, timestamp_ns))
