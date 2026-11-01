"""Gym tasks of flywheel_sim. Importing this package registers them.

With Isaac Lab's RL CLI, register them through the external callback:

    isaaclab train --rl_library rsl_rl --task Flywheel-Balance --external_callback flywheel_sim.tasks.register
    isaaclab play --rl_library rsl_rl --task Flywheel-Balance --external_callback flywheel_sim.tasks.register --viz newton

Tasks: Flywheel-Balance (keep the base upright on its foot tip with the wheel torque; Newton MJWarp),
Flywheel-Balance-PhysX (the same on PhysX).
"""

from . import balance  # noqa: F401  (registers the tasks)


def register() -> None:
    """--external_callback target: the tasks are registered by importing this package."""
