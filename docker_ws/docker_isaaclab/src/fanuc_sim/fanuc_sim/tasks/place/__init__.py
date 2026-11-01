import gymnasium as gym

gym.register(
    id="Fanuc-Place-Box",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.place_env_cfg:CrxPlaceEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:CrxPlacePPORunnerCfg",
    },
)
