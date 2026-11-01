import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg
from isaaclab.utils import configclass, replace

from fanuc_sim.objects import RED_BOX_CFG
from fanuc_sim.robots import crx5ia_2f85_cfg
from fanuc_sim.tasks.lift.mdp import (
    FLOOR_COLLIDER,
    FLOOR_CONTACT_LINKS,
    GRASP_LINKS,
    RED_BOX_PRIM,
    ActionsCfg,
    EventCfg,
    ObservationsCfg,
    RewardCfg,
    TerminationsCfg,
)

# env config

@configclass
class CrxLiftSceneCfg(InteractiveSceneCfg):
    # Robotパスは各環境に配置対応できるよう、ENV_REGEX_NSを使って正規表現で指定する. ex) envs/env_0/Robot
    robot: ArticulationCfg = replace(
        crx5ia_2f85_cfg("physx"),
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=replace(crx5ia_2f85_cfg("physx").spawn, activate_contact_sensors=True),  # for the contact sensors
    )

    red_box: RigidObjectCfg = RED_BOX_CFG

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0]),
        spawn=GroundPlaneCfg(color=(0.3, 0.3, 0.3)),  # darker tint (visual only)
    )
    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    def __post_init__(self):
        # floor contact sensors (scene names <link>_floor), read by mdp.floor_contact
        for name, path in FLOOR_CONTACT_LINKS.items():
            setattr(self, f"{name}_floor", ContactSensorCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Robot/{path}", filter_prim_paths_expr=[FLOOR_COLLIDER]
            ))
        # fingertip <-> red box contact sensors (scene names <link>_red_box), read by mdp grasp detection
        for name, path in GRASP_LINKS.items():
            setattr(self, f"{name}_red_box", ContactSensorCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Robot/{path}", filter_prim_paths_expr=[RED_BOX_PRIM]
            ))

# Env
@configclass
class CrxLiftEnvCfg(ManagerBasedRLEnvCfg):
    scene: CrxLiftSceneCfg = CrxLiftSceneCfg(num_envs = 4096, env_spacing = 2.5)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        self.decimation = 2
        self.episode_length_s = 10.0
        self.sim.dt = 0.01
