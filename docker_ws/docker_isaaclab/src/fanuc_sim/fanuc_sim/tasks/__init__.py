"""Gym tasks of fanuc_sim. Importing this package registers them.

With Isaac Lab's RL CLI, register them through the external callback:

    isaaclab train --rl_library rsl_rl --task Fanuc-Lift-Cube --external_callback fanuc_sim.tasks.register
    isaaclab play --rl_library rsl_rl --task Fanuc-Lift-Cube --external_callback fanuc_sim.tasks.register --viz kit

Tasks: Fanuc-Lift-Cube (lift the red box), Fanuc-Place-Box (put it on the fixed yellow box).
"""

from . import lift, place  # noqa: F401  (registers the tasks)


def register() -> None:
    """--external_callback target: the tasks are registered by importing this package."""
