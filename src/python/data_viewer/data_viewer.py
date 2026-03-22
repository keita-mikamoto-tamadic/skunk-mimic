from dora import Node
import struct
import time
from rich.live import Live
from rich.table import Table
from rich.console import Group
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

# ImuData: timestamp, ax, ay, az, gx, gy, gz, q0, q1, q2, q3, roll, pitch, yaw
# sizeof(ImuData) = 14 * sizeof(double) = 112
IMU_DATA_FMT = "<14d"  # little-endian
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FMT)  # 112

# LatencyData: can_avg_us, can_max_us, ctrl_avg_us, ctrl_max_us
LATENCY_FMT = "<4d"
LATENCY_SIZE = struct.calcsize(LATENCY_FMT)  # 32

# State enum with styles (C++ enum class State と対応)
STATE_STYLES = {
	0: ("OFF", "dim white"),
	1: ("STOP", "yellow bold"),
	2: ("READY", "blue bold"),
	3: ("RUN", "green bold")
}


def build_motor_table(axes, timestamp_ns=None, state=None, config=None):
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


def build_imu_table(imu_data):
	table = Table(show_header=True, title="IMU", title_style="cyan")
	table.add_column("", justify="left", style="dim")
	table.add_column("Roll", justify="right")
	table.add_column("Pitch", justify="right")
	table.add_column("Yaw", justify="right")
	table.add_column("ax", justify="right")
	table.add_column("ay", justify="right")
	table.add_column("az", justify="right")

	if imu_data is not None:
		# imu_data = (timestamp, ax, ay, az, gx, gy, gz, q0, q1, q2, q3, roll, pitch, yaw)
		roll, pitch, yaw = imu_data[11], imu_data[12], imu_data[13]
		ax, ay, az = imu_data[1], imu_data[2], imu_data[3]
		table.add_row(
			"[dim]rad/m/s²[/]",
			f"[cyan]{roll:+.3f}[/]",
			f"[cyan]{pitch:+.3f}[/]",
			f"[cyan]{yaw:+.3f}[/]",
			f"{ax:+.2f}",
			f"{ay:+.2f}",
			f"{az:+.2f}",
		)
	else:
		table.add_row("[dim]rad/m/s²[/]", "-", "-", "-", "-", "-", "-")

	return table


def build_latency_table(latency_data):
	table = Table(show_header=True, title="Latency", title_style="magenta")
	table.add_column("", justify="left", style="dim")
	table.add_column("avg", justify="right")
	table.add_column("max", justify="right")

	if latency_data is not None:
		can_avg, can_max, ctrl_avg, ctrl_max = latency_data
		total_avg = can_avg + ctrl_avg

		can_style = "green" if can_avg < 1500 else "yellow" if can_avg < 2500 else "red"
		ctrl_style = "green" if ctrl_avg < 1500 else "yellow" if ctrl_avg < 2500 else "red"
		total_style = "green" if total_avg < 2500 else "yellow" if total_avg < 3000 else "red"

		table.add_row("CAN",   f"[{can_style}]{can_avg:.0f}us[/]",   f"{can_max:.0f}us")
		table.add_row("CTRL",  f"[{ctrl_style}]{ctrl_avg:.0f}us[/]",  f"{ctrl_max:.0f}us")
		table.add_section()
		table.add_row("Total", f"[{total_style}]{total_avg:.0f}us[/]", "")
	else:
		table.add_row("CAN",  "-", "-")
		table.add_row("CTRL", "-", "-")

	return table


node = Node("data_viewer")

# Load robot configuration
config = robot_config.load_from_file(CONFIG_PATH)
print(f"Loaded config: {config.robot_name} ({config.axis_count} axes)")

initial_display = Group(
	build_motor_table([], config=config),
	build_imu_table(None),
	build_latency_table(None)
)

with Live(initial_display, refresh_per_second=30) as live:
	start_time = time.time_ns()
	current_state = None
	current_imu_data = None
	current_latency = None
	for event in node:
		if event["type"] == "INPUT":
			if event["id"] == "state_status":
				raw = bytes(event["value"].to_pylist())
				if len(raw) > 0:
					current_state = raw[0]
			elif event["id"] == "imu_data":
				raw = bytes(event["value"].to_pylist())
				if len(raw) >= IMU_DATA_SIZE:
					current_imu_data = struct.unpack(IMU_DATA_FMT, raw[:IMU_DATA_SIZE])
			elif event["id"] == "latency":
				raw = bytes(event["value"].to_pylist())
				if len(raw) >= LATENCY_SIZE:
					current_latency = struct.unpack(LATENCY_FMT, raw[:LATENCY_SIZE])
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

				# 両方のテーブルを更新
				motor_table = build_motor_table(axes, timestamp_ns, current_state, config)
				imu_table = build_imu_table(current_imu_data)
				latency_table = build_latency_table(current_latency)
				live.update(Group(motor_table, imu_table, latency_table))
