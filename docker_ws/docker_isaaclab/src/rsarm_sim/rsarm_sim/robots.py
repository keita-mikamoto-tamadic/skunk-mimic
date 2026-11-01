"""Isaac Lab configs for RSARM and its table.

Files are in robot_file/rsarm_v0 (mounted at /workspace/robot_file): the robot assembly rsarm.usd is built
once by ``scripts/build_rsarm_usd.py`` from the part USDs; the table wrapper table.usd
(mesh/table_assy.usd with its colliders split) is built by the same script.
Joints: shoulder_pitch (Y), shoulder_roll (X), shoulder_yaw (Z), elbow_pitch (Y), each rotating about the
child part origin.
"""

import os

from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.assets.articulation import ArticulationCfg

RSARM_DIR = os.environ.get("RSARM_DIR", "/workspace/robot_file/rsarm_v0")

RSARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{RSARM_DIR}/rsarm.usd",
        activate_contact_sensors=False,
        articulation_props=[
            # adjacent links touch at the joints; collisions with the table stay enabled
            PhysxArticulationCfg(enabled_self_collisions=False),
            NewtonArticulationCfg(self_collision_enabled=False),
        ],
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # elbow bent -90 deg about Y: forearm horizontal (+x). With q = 0 the hanging forearm touches the table beam.
        joint_pos={"shoulder_pitch": 0.0, "shoulder_roll": 0.0, "shoulder_yaw": 0.0, "elbow_pitch": -1.5708},
    ),
    actuators={
        # placeholders until the motor specs are known (links 0.12-0.76 kg, gravity torque < ~1 N m)
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=50.0,
            damping=2.0,
            joint_effort_limit=5.0,
            joint_velocity_limit=3.0,
        ),
    },
)
"""RSARM: A000 fixed to the world, A100..A400 serial (pitch, roll, yaw, pitch)."""

TABLE_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=f"{RSARM_DIR}/table.usd", scale=(1e-3, 1e-3, 1e-3)),  # mm -> m
    init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.148, -0.290)),  # "World X,Y,Z" of frame_base.csv
)
"""Static table (colliders without rigid bodies), placed in the same frame as RSARM."""

# top face of the table plate at the blue box (its collider Cube too) [m]
TABLE_TOP_Z = -0.290
BLUE_BOX_SIZE = 0.05

BLUE_BOX_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/BlueBox",
    spawn=sim_utils.CuboidCfg(
        size=(BLUE_BOX_SIZE,) * 3,
        rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
        collision_props=sim_utils.UsdPhysicsCollisionCfg(),
        mass_props=sim_utils.MassCfg(mass=0.1),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.8),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.05, 0.2, 0.8)),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(0.2, -0.05, TABLE_TOP_Z + BLUE_BOX_SIZE / 2)),
)
"""50 mm blue cube resting on the table at x = 200 mm, y = -200 mm (robot frame)."""
