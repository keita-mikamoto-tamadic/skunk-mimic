"""Scene and env of placing the red box on the fixed yellow box: the lift task's scene and env with the yellow box
added, and the place MDP (mdp.py)."""

from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from fanuc_sim.objects import YELLOW_BOX_CFG
from fanuc_sim.tasks.lift.lift_env_cfg import CrxLiftEnvCfg, CrxLiftSceneCfg
from fanuc_sim.tasks.place.mdp import (
    PLAY_PHASE_WEIGHTS,
    YELLOW_BOX_PRIM,
    YELLOW_CONTACT_LINKS,
    EventCfg,
    ObservationsCfg,
    RewardCfg,
    TerminationsCfg,
)


@configclass
class CrxPlaceSceneCfg(CrxLiftSceneCfg):
    # kinematic: fixed in place, the red box and the robot still collide with it
    yellow_box: RigidObjectCfg = YELLOW_BOX_CFG

    def __post_init__(self):
        super().__post_init__()
        # robot link <-> yellow box contact sensors (scene names <link>_yellow_box), read by mdp.yellow_box_contact
        for name, path in YELLOW_CONTACT_LINKS.items():
            setattr(self, f"{name}_yellow_box", ContactSensorCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Robot/{path}", filter_prim_paths_expr=[YELLOW_BOX_PRIM]
            ))


@configclass
class CrxPlaceEnvCfg(CrxLiftEnvCfg):
    scene: CrxPlaceSceneCfg = CrxPlaceSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        super().__post_init__()
        # grasp, carry over and set down: longer than the lift
        self.episode_length_s = 15.0

    def play_mode(self):
        """Play and evaluate whole episodes: start every one from home, not from a recorded mid-task phase."""
        super().play_mode()
        self.events.reset_from_states.params["phase_weights"] = PLAY_PHASE_WEIGHTS
