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

# wheel motor — identified on the real rig 2026-11-01 (skunk-mimic sysid/flywheel/, fit_result.json in
# robot_file/fly_wheel/). The FOCTIVE impedance loop outputs CURRENT, so the torque limit is the current
# clamp: tau_max 0.38 N m. Gravity torque is m g l sin(tilt) = 0.38 N m x sin(tilt) (0.60 kg, center of
# mass 64 mm above the tip) — the motor torque only matches gravity's coefficient, so balance works near
# upright but large tilts are unrecoverable by torque alone (wheel momentum is the real budget:
# 0 -> 30 rad/s stores (4.3e-4 + 6.8e-4 armature) x 30 = 3.3e-2 N m s).
# The current clamp is flat in speed (the sysid fit used a flat torque clamp and matched up to
# 16 rad/s), so it belongs in actuator_effort_limit (continuous torque), NOT saturation_effort:
# DCMotor clips to min(effort_limit, stall line), giving flat 0.38 up to the ~29 rad/s corner.
WHEEL_MAX_TORQUE = 0.38  # [N m]    identified current-limit torque
WHEEL_STALL_TORQUE = 10.0  # [N m]  electrical stall (shapes the curve only near no-load speed)
WHEEL_NO_LOAD_SPEED = 30.0  # [rad/s] (firmware velocity_limit is 33)
WHEEL_ARMATURE = 6.8e-4  # [kg m^2]  rotor x gear^2, identified
WHEEL_FRICTION = 0.080  # [N m]      Coulomb, identified
WHEEL_VISCOUS = 0.0138  # [N m s/rad] viscous, identified
# real position-loop PD at the output shaft (imp_kp=30 A/rad x current->torque chain), identified:
WHEEL_KP = 8.152  # [N m/rad]
WHEEL_KD = 0.0544  # [N m s/rad]

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
            actuator_effort_limit=WHEEL_MAX_TORQUE,
            actuator_velocity_limit=WHEEL_NO_LOAD_SPEED,
            armature=WHEEL_ARMATURE,
            friction=WHEEL_FRICTION,
            viscous_friction=WHEEL_VISCOUS,
        ),
    },
)
"""Flywheel robot: base free on the floor (z = 0), wheel joint about X at the base origin, driven by torque."""
