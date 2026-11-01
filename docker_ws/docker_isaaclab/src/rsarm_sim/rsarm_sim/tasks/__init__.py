"""Gym tasks of rsarm_sim. Importing this package registers them.

With Isaac Lab's RL CLI, register them through the external callback:

    isaaclab train --rl_library rsl_rl --task Rsarm-Push-Box --external_callback rsarm_sim.tasks.register
    isaaclab play --rl_library rsl_rl --task Rsarm-Push-Box --external_callback rsarm_sim.tasks.register --viz newton

Tasks: Rsarm-Push-Box (push the blue box over the table to a target point with the tcp sphere; actions: a tcp
goal, Newton IK, Newton physics), Rsarm-Push-Box-PhysX (the same on PhysX, differential IK),
Rsarm-Push-Box-Joint (Newton, interpolated joint targets as actions).
"""

from . import push  # noqa: F401  (registers the tasks)


def register() -> None:
    """--external_callback target: the tasks are registered by importing this package."""
