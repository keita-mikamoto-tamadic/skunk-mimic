"""RSL-RL PPO for placing the red box on the yellow box: the lift task's settings, a longer run."""

from isaaclab.utils import configclass

from fanuc_sim.tasks.lift.agents.rsl_rl_ppo_cfg import CrxLiftPPORunnerCfg


@configclass
class CrxPlacePPORunnerCfg(CrxLiftPPORunnerCfg):
    max_iterations = 3000
    experiment_name = "crx_place"
