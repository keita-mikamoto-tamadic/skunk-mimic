"""RSL-RL PPO for lifting the red box. Settings of Isaac Lab's SO-101 vial tutorial (IsaacLabTutorial, place_vial,
agents/rsl_rl_ppo_cfg.py: SO101StatePPORunnerCfg)."""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class BoundedGaussianDistributionCfg(RslRlMLPModelCfg.GaussianDistributionCfg):
    """Gaussian policy head whose standard deviation is clamped to ``std_range`` (rsl_rl GaussianDistribution).

    Without a cap the entropy bonus inflates the std once actions are clipped to [-1, 1], and noise that large
    random-walks the gripper so the approach cannot be learned (the tutorial's finding).
    """

    std_range: tuple[float, float] = (0.05, 0.3)


@configclass
class CrxLiftPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    seed = 42
    num_steps_per_env = 64
    max_iterations = 800
    save_interval = 50
    experiment_name = "crx_lift"
    clip_actions = 1.0
    # asymmetric actor-critic: the critic also sees the privileged "critic" group
    obs_groups = {"actor": ["policy"], "critic": ["policy", "critic"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=BoundedGaussianDistributionCfg(init_std=0.2, std_type="log"),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[256, 256, 128], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=8,
        learning_rate=3.0e-4,
        schedule="adaptive",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
