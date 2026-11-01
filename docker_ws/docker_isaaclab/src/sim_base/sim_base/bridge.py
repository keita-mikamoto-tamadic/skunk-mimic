"""Bridges between the sim loop and the outside."""

import argparse
import math


class Bridge:
    """Joint commands in, joint states out. This base talks to no one: the robot holds its initial pose, and Python
    code (an app hook, a test) can set :attr:`target` / :attr:`effort` directly.

    Subclasses connect a transport (:class:`~sim_base.ros_bridge.RosBridge`, :class:`~sim_base.dora_bridge.DoraBridge`) and feed
    :meth:`set_command` from it. Names on the outside are ``name_map[isaac_name]`` (default: the Isaac name),
    with ``prefix`` prepended.
    """

    name = "none"

    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        """Bridge-specific command line options."""

    def __init__(self, args: argparse.Namespace, joint_names: list[str], name_map: dict[str, str] | None = None,
                 prefix: str = ""):
        self.args = args
        name_map = name_map or {}
        self.names = [prefix + name_map.get(n, n) for n in joint_names]
        self._index = {n: i for i, n in enumerate(self.names)}
        # position target per joint (Isaac order); set by the app to the initial pose before start()
        self.target: list[float] = []
        # latest effort command per joint index, for apps that interpret it (e.g. a gripper force)
        self.effort: dict[int, float] = {}

    def set_command(self, names, positions, efforts=()) -> None:
        """Apply commands given by outside names. Non-finite values and unknown names are ignored. Efforts count
        only when there is one per name."""
        for name, pos in zip(names, positions):
            i = self._index.get(name)
            if i is not None and math.isfinite(pos):
                self.target[i] = pos
        if len(efforts) == len(names):
            for name, effort in zip(names, efforts):
                i = self._index.get(name)
                if i is not None and math.isfinite(effort):
                    self.effort[i] = effort

    def start(self) -> None:
        """Start receiving (called once, after :attr:`target` holds the initial pose)."""

    def publish(self, sim_time: float, pos, vel, effort=None) -> None:
        """Send the state after a physics step (``pos`` / ``vel`` / ``effort``: env 0 tensors in Isaac joint order;
        ``effort`` is the joint torque the drives applied)."""

    def shutdown(self) -> None:
        pass
