"""Common Isaac Lab sim app for the robots in isaaclab_ws.

A robot script subclasses :class:`SimApp` and overrides only what is robot specific (the articulation, extra
scene assets, Newton requirements, per-step hooks). The bridge to the outside is pluggable: :class:`Bridge` talks to
no one (the robot holds its pose, or Python sets targets), :class:`RosBridge` exchanges joint states / commands
over ROS 2 (``--bridge ros``), :class:`DoraBridge` skunk-mimic's axis structs over dora (``--bridge dora``).
"""

from .app import SimApp
from .bridge import Bridge
from .control import EffortLimitedDrive
from .physics import physics_cfg

__all__ = ["Bridge", "EffortLimitedDrive", "SimApp", "physics_cfg"]
