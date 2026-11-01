"""Articulation configs for FANUC robots on the NVIDIA asset server.

Assets: {ISAAC_NUCLEUS_DIR}/Robots/Fanuc/<model>/<model>.usd
Joint names are J1..J6, matching fanuc_driver (without prefix).
"""

import math
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils import replace
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

ROBOT_FILE_DIR = os.environ.get("ROBOT_FILE_DIR", "/workspace/robot_file")


CRX5IA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Fanuc/crx5ia/crx5ia.usd",
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # Ready pose: flange (J6 axis) pointing straight down, J5 = -90 deg (farthest from the J5 = 0 wrist
        # singularity of the all-zero pose), J6 rolled -90 deg.
        # Keep equal to home_positions in ros2_ws/src/xr_teleop_control/config/clutch_teleop.yaml.
        joint_pos={"J1": 0.0, "J2": -0.3, "J3": -0.3, "J4": 0.0, "J5": -1.5708, "J6": -1.5708},
    ),
    actuators={
        # None: keep the drive gains / effort limits authored in the USD
        # (J1-J3: 160 N·m, J4-J6: 40 N·m)
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["J[1-6]"],
            stiffness=None,
            damping=None,
        ),
    },
)
"""FANUC CRX-5iA (fixed base, position-controlled)."""

CRX5IA_2F85_CFG = replace(
    CRX5IA_CFG,
    spawn=sim_utils.UsdFileCfg(
        # built once by scripts/build_crx5ia_2f85_usd.py; variant "Gripper" = PhysX | Newton (crx5ia_2f85_cfg())
        usd_path=f"{ROBOT_FILE_DIR}/crx5ia_2f85/crx5ia_2f85.usd",
        activate_contact_sensors=False,
        variants={"Gripper": "PhysX"},
    ),
    init_state=replace(CRX5IA_CFG.init_state, joint_pos={**CRX5IA_CFG.init_state.joint_pos, "finger_joint": 0.0}),
    actuators={
        **CRX5IA_CFG.actuators,
        # finger_joint drives the gripper (0 = open, ~0.8 rad = closed); the other finger joints follow it
        # through the asset's own mimic joints / couplings, so they get no actuator. Gains from the USD.
        "gripper": ImplicitActuatorCfg(joint_names_expr=["finger_joint"], stiffness=None, damping=None),
    },
)
"""CRX-5iA with a Robotiq 2F-85 on the flange (PhysX settings; use :func:`crx5ia_2f85_cfg` per engine)."""


# Effective inertia each arm joint drives with the 2F-85 on [kg m^2]: diagonal of the joint-space mass matrix, the
# larger of the home and the stretched (J2 = J3 = J5 = 0) pose. Rotor inertia is not in the asset (armature 0).
ARM_INERTIA_2F85 = {"J1": 1.80, "J2": 3.86, "J3": 1.59, "J4": 0.126, "J5": 0.059, "J6": 0.0032}
# Newton arm: the same critically damped response on every joint, kp = I wn^2, kd = 2 I wn.
# 15 Hz is stable up to dt 0.01 (step 0.3 rad: settles, sag <= 5 mrad on J4); 25 Hz oscillates at dt 0.01.
NEWTON_ARM_WN_HZ = 15.0


def actuators_for(engine: str) -> dict[str, ImplicitActuatorCfg]:
    """Arm and gripper actuator gains per physics engine ("physx" or "newton_*"). None = the value in the USD."""
    if engine.startswith("newton"):
        # The USD arm gains (kp 1.43e5, kd 5.7e3 on every joint, so the wrist is ~1000x stiffer per inertia
        # than the base) oscillate on MJWarp at dt 0.01. Scale the gains with each joint's inertia instead.
        wn = 2.0 * math.pi * NEWTON_ARM_WN_HZ
        arm = dict(
            stiffness={j: i * wn**2 for j, i in ARM_INERTIA_2F85.items()},
            damping={j: 2.0 * i * wn for j, i in ARM_INERTIA_2F85.items()},
        )
        finger = dict(stiffness=None, damping=None)  # Robotiq's Newton tuning in the asset
    else:
        arm = dict(stiffness=None, damping=None)
        finger = dict(stiffness=None, damping=None)
    return {
        "arm": ImplicitActuatorCfg(joint_names_expr=["J[1-6]"], **arm),
        "gripper": ImplicitActuatorCfg(joint_names_expr=["finger_joint"], **finger),
    }


def crx5ia_2f85_cfg(engine: str) -> ArticulationCfg:
    """CRX5IA_2F85_CFG with the gripper physics variant and the actuator gains for ``engine``."""
    variant = "Newton" if engine.startswith("newton") else "PhysX"
    return replace(
        CRX5IA_2F85_CFG,
        spawn=replace(CRX5IA_2F85_CFG.spawn, variants={"Gripper": variant}),
        actuators=actuators_for(engine),
    )


# ---- Robotiq 2F-85 -------------------------------------------------------------------------------------------------

ROBOTIQ_DRIVER_JOINT_NAMES = {"finger_joint": "robotiq_85_left_knuckle_joint"}
"""Isaac joint name -> joint name of the real Robotiq driver (ros2_robotiq_gripper), so ROS runs unchanged on it."""

ROBOTIQ_NEWTON_IMPRATIO = 10.0
"""Anti-slip setting of the Robotiq Newton asset (its newton/apply_gripper_tuning.py)."""

ROBOTIQ_NEWTON_MAX_SOLVER_DT = 0.002
"""The asset's loop-closure equalities are tuned for 500 Hz; MuJoCo softens a constraint whose time constant is
below 2 solver steps, and at 0.01 s the left four-bar folds over (-1.87 rad) under gravity."""

ROBOTIQ_FORCE_MIN, ROBOTIQ_FORCE_MAX = 20.0, 235.0


def robotiq_force_fraction(effort: float) -> float:
    """Grip force command [N] of the real driver -> fraction of the maximum force.

    The driver scales |effort| / 235 to the force register (0..255), which spans about 20..235 N on the gripper.
    """
    frac = min(abs(effort) / ROBOTIQ_FORCE_MAX, 1.0)
    return (ROBOTIQ_FORCE_MIN + frac * (ROBOTIQ_FORCE_MAX - ROBOTIQ_FORCE_MIN)) / ROBOTIQ_FORCE_MAX
