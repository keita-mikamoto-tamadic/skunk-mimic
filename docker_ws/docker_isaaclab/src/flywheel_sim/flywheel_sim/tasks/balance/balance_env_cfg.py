"""Scene and env of balancing the flywheel robot (the MDP is in mdp.py)."""

import isaaclab.sim as sim_utils
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg

from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass, replace

from flywheel_sim.robots import FLYWHEEL_CFG
from flywheel_sim.tasks.balance.mdp import ActionsCfg, EventCfg, ObservationsCfg, RewardCfg, TerminationsCfg
from sim_base.physics import physics_cfg


@configclass
class FlywheelBalanceSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg = replace(FLYWHEEL_CFG, prim_path="{ENV_REGEX_NS}/Robot")
    # friction 0.5 combined by max (MuJoCo always takes the max): the randomized foot friction (>= 0.5, mdp.py) acts
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            color=(0.3, 0.3, 0.3),
            physics_material=PhysxRigidBodyMaterialCfg(
                static_friction=0.5, dynamic_friction=0.5, friction_combine_mode="max"
            ),
        ),
    )
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=3000.0))


@configclass
class FlywheelBalanceEnvCfg(ManagerBasedRLEnvCfg):
    scene: FlywheelBalanceSceneCfg = FlywheelBalanceSceneCfg(num_envs=4096, env_spacing=0.5)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        # policy at 100 Hz over 0.5 ms physics steps: the DC motor model's torque limit follows the wheel speed once
        # per step, and at 5 N m the wheel gains 15 rad/s in 0.5 ms (of its 30 rad/s no-load speed, robots.py)
        self.sim.dt = 0.0005
        self.decimation = 20
        self.sim.physics = physics_cfg("newton_mjwarp", self.sim.dt)
        # stiffer contacts than Newton's default (21 ms time constant, the foot sinks by mm): MJWarp maps ke/kd to
        # time constant 2 / (0.95 kd) = 3 ms (>= 2 solver steps) and damping ratio kd / 2 * sqrt(0.95 / ke) = 1
        self.sim.physics.default_shape_cfg.ke = 1.17e5
        self.sim.physics.default_shape_cfg.kd = 702.0
        self.sim.render_interval = self.decimation
        self.episode_length_s = 10.0
        self.viewer.eye = (0.4, 0.4, 0.25)
        self.viewer.lookat = (0.0, 0.0, 0.06)


@configclass
class FlywheelBalancePhysxEnvCfg(FlywheelBalanceEnvCfg):
    """The same task on PhysX."""

    def __post_init__(self):
        super().__post_init__()
        self.sim.physics = PhysxCfg()
