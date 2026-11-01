import gymnasium as gym

gym.register(
    id="Rsarm-Push-Box",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.push_env_cfg:RsarmPushEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:RsarmPushPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Rsarm-Push-Box-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.push_env_cfg:RsarmPushJointEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:RsarmPushPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Rsarm-Push-Box-PhysX",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.push_env_cfg:RsarmPushPhysxEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:RsarmPushPhysxPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
