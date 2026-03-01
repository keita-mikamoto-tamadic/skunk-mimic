"""Robot configuration loader (Python port of robot_config.cpp)."""

from dataclasses import dataclass
from typing import List
import json
import math


@dataclass
class AxisConfig:
    """Configuration for a single axis (motor)."""
    index: int
    name: str
    device_id: int
    motdir: int
    initial_position: float
    reset_position: float
    velocity_limit: float
    accel_limit: float
    torque_limit: float


@dataclass
class RobotConfig:
    """Robot configuration."""
    robot_name: str
    axis_count: int
    interpolation_time: float
    transport: str  # "socketcan" or "dummy"
    axes: List[AxisConfig]


def _parse_double(value) -> float:
    """Parse double with NaN support for string values."""
    if isinstance(value, str):
        return math.nan
    return float(value)


def parse(json_str: str) -> RobotConfig:
    """Parse JSON string to RobotConfig.

    Args:
        json_str: JSON string containing robot configuration

    Returns:
        RobotConfig object
    """
    data = json.loads(json_str)

    axes = []
    for ax in data["axes"]:
        axes.append(AxisConfig(
            index=ax["index"],
            name=ax["name"],
            device_id=ax["device_id"],
            motdir=ax["motdir"],
            initial_position=ax["initial_position"],
            reset_position=ax["reset_position"],
            velocity_limit=_parse_double(ax["velocity_limit"]),
            accel_limit=_parse_double(ax["accel_limit"]),
            torque_limit=ax["torque_limit"]
        ))

    return RobotConfig(
        robot_name=data["robot_name"],
        axis_count=data["axis_count"],
        interpolation_time=data["interpolation_time"],
        transport=data.get("transport", "socketcan"),
        axes=axes
    )


def load_from_file(path: str) -> RobotConfig:
    """Load RobotConfig from JSON file.

    Args:
        path: Path to JSON configuration file

    Returns:
        RobotConfig object
    """
    with open(path, "r", encoding="utf-8") as f:
        return parse(f.read())
