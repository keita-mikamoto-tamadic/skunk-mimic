"""Parameter definitions for wheel system identification.

Defines optimizable parameters with left-right symmetry constraint:
  - armature: rotor reflected inertia
  - damping: viscous friction
  - frictionloss: Coulomb friction
  - ground_friction: wheel-ground sliding friction

Left/right constraint: wheel_r params = wheel_l params (4 free parameters).
"""

import mujoco
import numpy as np
from mujoco import sysid


def _make_joint_modifier(joint_name: str, attr: str):
    """Create a modifier function that sets a joint attribute via MjSpec."""

    def modifier(spec: mujoco.MjSpec, param: sysid.Parameter):
        for j in spec.joints:
            if j.name == joint_name:
                setattr(j, attr, param.value)
                break
        return spec

    return modifier


def _make_symmetric_joint_modifier(joint_names: list[str], attr: str):
    """Create a modifier that sets the same value on multiple joints."""

    def modifier(spec: mujoco.MjSpec, param: sysid.Parameter):
        val = float(np.asarray(param.value).flat[0])
        for j in spec.joints:
            if j.name in joint_names:
                setattr(j, attr, val)
        return spec

    return modifier


def _make_ground_friction_modifier():
    """Create a modifier that sets the ground plane geom's slide friction.

    The ground plane is the first geom (unnamed, type=plane) in worldbody.
    MuJoCo friction array: [slide, spin, roll].
    """

    def modifier(spec: mujoco.MjSpec, param: sysid.Parameter):
        for g in spec.geoms:
            if g.type == mujoco.mjtGeom.mjGEOM_PLANE:
                friction = list(g.friction)
                friction[0] = float(np.asarray(param.value).flat[0])
                g.friction = friction
                break
        return spec

    return modifier


def build_params() -> sysid.ParameterDict:
    """Build ParameterDict for wheel system identification.

    Returns 4 parameters with left-right symmetry:
      wheel_armature: applied to wheel_R_joint and wheel_L_joint
      wheel_damping: applied to wheel_R_joint and wheel_L_joint
      wheel_frictionloss: applied to wheel_R_joint and wheel_L_joint
      ground_friction: applied to ground plane geom
    """
    wheel_joints = ["wheel_R_joint", "wheel_L_joint"]

    params = sysid.ParameterDict({
        "wheel_armature": sysid.Parameter(
            name="wheel_armature",
            nominal=0.05,       # current value in mimic_v2.xml
            min_value=0.0001,
            max_value=0.5,
            modifier=_make_symmetric_joint_modifier(wheel_joints, "armature"),
        ),
        "wheel_damping": sysid.Parameter(
            name="wheel_damping",
            nominal=0.0,
            min_value=0.0,
            max_value=2.0,
            modifier=_make_symmetric_joint_modifier(wheel_joints, "damping"),
        ),
        "wheel_frictionloss": sysid.Parameter(
            name="wheel_frictionloss",
            nominal=0.0,
            min_value=0.0,
            max_value=2.0,
            modifier=_make_symmetric_joint_modifier(wheel_joints, "frictionloss"),
        ),
        "ground_friction": sysid.Parameter(
            name="ground_friction",
            nominal=1.0,
            min_value=0.1,
            max_value=5.0,
            modifier=_make_ground_friction_modifier(),
        ),
    })

    return params
