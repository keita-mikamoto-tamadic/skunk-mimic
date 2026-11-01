import gymnasium as gym

gym.register(
    id = "Fanuc-Lift-Cube",
    entry_point = "isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker = True,
    kwargs = {
      "env_cfg_entry_point" : f"{__name__}.lift_env_cfg:CrxLiftEnvCfg",
      "rsl_rl_cfg_entry_point" : f"{__name__}.agents.rsl_rl_ppo_cfg:CrxLiftPPORunnerCfg",
    },
)