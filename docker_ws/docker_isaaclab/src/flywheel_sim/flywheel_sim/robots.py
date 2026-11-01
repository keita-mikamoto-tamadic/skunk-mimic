"""Isaac Lab config of the flywheel robot: a reaction-wheel inverted pendulum.

Files are in robot_file/fly_wheel (mounted at /workspace/robot_file): flywheel.usd is built once by
``scripts/build_flywheel_usd.py`` from the part USDs. The base stands on the rounded tip of its foot, 62.4 mm below
the part origin, and tips over about X (its foot is a 43 mm line along X, so it cannot fall about Y); the wheel
spins about X on the joint ``wheel`` at the origin, and its reaction torque balances the base. Fallen, the base
rests on a side corner at about 45 deg.
"""

import os

from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

FLYWHEEL_DIR = os.environ.get("FLYWHEEL_DIR", "/workspace/robot_file/fly_wheel")

# tip of the foot below the part origin [m] (lowest point of the base collider)
FOOT_DEPTH = 0.06235

# wheel motor. Gravity torque is m g l sin(tilt) = 0.38 N m x sin(tilt) (0.60 kg, center of mass 64 mm above the
# tip), so the torque is ample; the speed is the limit: 0 -> 30 rad/s stores 1.28e-2 N m s in the wheel
# (4.3e-4 kg m^2), enough to bring the base back from about 0.37 rad at rest. At full torque the wheel would reach
# 30 rad/s in 1.3 ms, faster than the speed-dependent torque limit can follow at the 0.5 ms physics step; the task
# commands at most 0.5 N m (balance/mdp.py ACTION_TORQUE), which the step resolves.
WHEEL_STALL_TORQUE = 10.0  # [N m]
WHEEL_NO_LOAD_SPEED = 30.0  # [rad/s]

FLYWHEEL_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{FLYWHEEL_DIR}/flywheel.usd",
        activate_contact_sensors=False,
        articulation_props=[
            # base and wheel overlap at the hub and are joint-connected
            PhysxArticulationCfg(enabled_self_collisions=False),
            NewtonArticulationCfg(self_collision_enabled=False),
        ],
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, FOOT_DEPTH), joint_pos={"wheel": 0.0}),
    actuators={
        # torque control (no PD): the DC motor model cuts the torque linearly to 0 at the no-load speed
        "wheel": DCMotorCfg(
            joint_names_expr=["wheel"],
            stiffness=0.0,
            damping=0.0,
            saturation_effort=WHEEL_STALL_TORQUE,
            actuator_effort_limit=WHEEL_STALL_TORQUE,
            actuator_velocity_limit=WHEEL_NO_LOAD_SPEED,
        ),
    },
)
"""Flywheel robot: base free on the floor (z = 0), wheel joint about X at the base origin, driven by torque."""
