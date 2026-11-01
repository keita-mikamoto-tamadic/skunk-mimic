import gymnasium as gym

gym.register(
    id="Flywheel-Balance",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.balance_env_cfg:FlywheelBalanceEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:FlywheelBalancePPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Flywheel-Balance-PhysX",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.balance_env_cfg:FlywheelBalancePhysxEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:FlywheelBalancePhysxPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
