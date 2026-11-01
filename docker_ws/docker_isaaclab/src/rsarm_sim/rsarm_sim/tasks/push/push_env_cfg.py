"""Scene and env of pushing the blue box to the target on the RSARM table (the MDP is in mdp.py)."""

import isaaclab.sim as sim_utils
from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg

from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass, replace

from rsarm_sim.robots import BLUE_BOX_CFG, RSARM_CFG, TABLE_CFG
from sim_base.physics import physics_cfg
from rsarm_sim.tasks.push.mdp import (
    CONTACT_SENSORS,
    SIMPLE_TABLE,
    SIMPLE_TABLE_COLLIDERS,
    ActionsCfg,
    EventCfg,
    JointActionsCfg,
    PhysxActionsCfg,
    ObservationsCfg,
    RewardCfg,
    TerminationsCfg,
)


@configclass
class RsarmPushSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg = replace(
        RSARM_CFG,
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=replace(
            RSARM_CFG.spawn,
            activate_contact_sensors=True,
            # self collision on, so links cannot pass through each other (joint-connected pairs are skipped)
            articulation_props=[NewtonArticulationCfg(self_collision_enabled=True)],
        ),
    )
    table: AssetBaseCfg = replace(TABLE_CFG, prim_path="{ENV_REGEX_NS}/Table")
    blue_box: RigidObjectCfg = BLUE_BOX_CFG

    # floor under the table (its legs stand on it)
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, -1.188)),
        spawn=sim_utils.GroundPlaneCfg(color=(0.3, 0.3, 0.3)),
    )
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=3000.0))

    def __post_init__(self):
        # contact sensors read by the mdp (scene names in CONTACT_SENSORS). The table has no rigid body: its colliders
        # are filtered as shapes, the robot links and the box as bodies.
        for name, (link, filters) in CONTACT_SENSORS.items():
            paths = [f"{{ENV_REGEX_NS}}/{p}" for p in filters]
            shapes = name.endswith("_table")
            setattr(self, name, ContactSensorCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Robot/{link}",
                filter_prim_paths_expr=[] if shapes else paths,
                filter_shape_prim_expr=paths if shapes else [],
            ))


@configclass
class RsarmPushEnvCfg(ManagerBasedRLEnvCfg):
    scene: RsarmPushSceneCfg = RsarmPushSceneCfg(num_envs=4096, env_spacing=2.0)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        # policy at 50 Hz over 5 ms physics steps (the IK action interpolates the tcp path over them). Newton MJWarp
        # (its IK needs Newton physics), each step split into 1 ms solver steps
        self.sim.dt = 0.005
        self.decimation = 4
        self.sim.physics = physics_cfg("newton_mjwarp", self.sim.dt, max_solver_dt=0.001)
        # stiffer contacts than Newton's default (21 ms time constant, the box sinks by mm): MJWarp maps ke/kd to
        # time constant 2 / (0.95 kd) = 3 ms (>= 2 solver steps) and damping ratio kd / 2 * sqrt(0.95 / ke) = 1
        self.sim.physics.default_shape_cfg.ke = 1.17e5
        self.sim.physics.default_shape_cfg.kd = 702.0
        self.sim.render_interval = self.decimation
        self.episode_length_s = 10.0


@configclass
class RsarmPushJointEnvCfg(RsarmPushEnvCfg):
    """The same task with interpolated joint targets as actions instead of the tcp goal (no IK)."""

    actions: JointActionsCfg = JointActionsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.observations.policy = ObservationsCfg.PolicyCfg()


@configclass
class RsarmPushPhysxSceneCfg(RsarmPushSceneCfg):
    robot: ArticulationCfg = replace(
        RSARM_CFG,
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=replace(
            RSARM_CFG.spawn,
            activate_contact_sensors=True,
            articulation_props=[PhysxArticulationCfg(enabled_self_collisions=True)],
        ),
    )
    # the table as two static cuboids (mdp.SIMPLE_TABLE), not the table USD
    table: AssetBaseCfg | None = None

    def __post_init__(self):
        super().__post_init__()
        for name, center, size in SIMPLE_TABLE:
            setattr(self, name, AssetBaseCfg(
                prim_path=f"{{ENV_REGEX_NS}}/{name}",
                init_state=AssetBaseCfg.InitialStateCfg(pos=center),
                spawn=sim_utils.CuboidCfg(
                    size=size,
                    collision_props=sim_utils.UsdPhysicsCollisionCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.55, 0.55)),
                ),
            ))
        # PhysX filters on collider prims directly (no shape-level fields); the table sensors on the cuboids
        for name, (link, filters) in CONTACT_SENSORS.items():
            filters = SIMPLE_TABLE_COLLIDERS if name.endswith("_table") else filters
            setattr(self, name, ContactSensorCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Robot/{link}",
                filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/{p}" for p in filters],
            ))


@configclass
class RsarmPushPhysxEnvCfg(RsarmPushEnvCfg):
    """The same task on PhysX (stiffer contacts than MuJoCo's soft ones for pushing), with Isaac Lab's differential
    IK instead of Newton IK."""

    scene: RsarmPushPhysxSceneCfg = RsarmPushPhysxSceneCfg(num_envs=4096, env_spacing=2.0)
    actions: PhysxActionsCfg = PhysxActionsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.sim.physics = PhysxCfg()
