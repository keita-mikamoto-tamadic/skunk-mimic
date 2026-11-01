"""Primitive objects for the CRX-5iA tasks: a red box to pick and a yellow box to stack it on.

Both rest on the ground in front of the robot (base at the origin, J1 = 0 faces +x): the red box is a free rigid
body, the yellow box a kinematic one (fixed in place).
The 2F-85 opens to 85 mm, so the red box (50 mm) fits between the fingers with margin.
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

RED_BOX_SIZE = (0.05, 0.05, 0.05)
YELLOW_BOX_SIZE = (0.16, 0.16, 0.16)


def box_cfg(
    name: str,
    size: tuple[float, float, float],
    color: tuple[float, float, float],
    mass: float,
    pos: tuple[float, float],
    kinematic: bool = False,
):
    """A box ``name`` (prim under the env) of ``size`` [m] resting on the ground at ``pos`` (x, y) [m].
    ``kinematic``: fixed in place (no gravity, not pushed), other bodies still collide with it."""
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        spawn=sim_utils.CuboidCfg(
            size=size,
            rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=kinematic),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            mass_props=sim_utils.MassCfg(mass=mass),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(*pos, size[2] / 2)),
    )


RED_BOX_CFG = box_cfg("RedBox", RED_BOX_SIZE, color=(0.8, 0.05, 0.05), mass=0.1, pos=(0.5, -0.15))
"""Box to pick."""

YELLOW_BOX_CFG = box_cfg(
    "YellowBox", YELLOW_BOX_SIZE, color=(0.9, 0.75, 0.05), mass=0.5, pos=(0.3, 0.10), kinematic=True
)
"""Box to place the red box on, fixed in place."""

ASSET_SETS = {
    "boxes": {"red_box": RED_BOX_CFG, "yellow_box": YELLOW_BOX_CFG},
}
"""Scene assets besides the robot, by set name then scene name (sim_bridge.py --assets <set>)."""
