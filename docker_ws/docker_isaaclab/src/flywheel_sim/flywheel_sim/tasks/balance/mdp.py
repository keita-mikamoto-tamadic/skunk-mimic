"""MDP of balancing the flywheel robot upright on its foot tip with the wheel torque.

Observations are what the robot can sense itself (IMU tilt and gyro about X, wheel encoder velocity); the policy outputs
the wheel torque at 100 Hz. The reward pays for staying alive, upright and still (positive kernels, so living always
beats falling), and penalizes wheel speed, torque and its rate a little. Episodes start tilted and spinning about the
foot tip, and pushes about the tip come every few seconds, both within the angular momentum the geared wheel can take
up (robots.py).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

import isaaclab.utils.math as math_utils
from isaaclab.envs import mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.noise import GaussianNoiseCfg as Gnoise

from flywheel_sim.robots import FOOT_DEPTH, WHEEL_NO_LOAD_SPEED

if TYPE_CHECKING:
    # not at runtime: importing the env classes loads USD before Kit starts (PhysX runs then fail to launch)
    from isaaclab.envs import ManagerBasedEnv

# tilt beyond which the base counts as fallen [rad] (it lies on its side corner at about 45 deg)
FALL_TILT = 0.5

# wheel torque at action 1 [N m] = the identified current-limit torque of the real motor
# (robots.WHEEL_MAX_TORQUE, 0.38 N m); the actuator clamps there anyway, so a larger scale
# would only flat-top the policy's actions
ACTION_TORQUE = 0.38

WHEEL = SceneEntityCfg("robot", joint_names=["wheel"])


def _tip_to_origin_w(quat_w: torch.Tensor) -> torch.Tensor:
    """Vector from the foot tip to the base origin in the world frame."""
    up = torch.tensor([0.0, 0.0, FOOT_DEPTH], device=quat_w.device).expand(quat_w.shape[0], 3)
    return math_utils.quat_apply(quat_w, up)


def _write_tilt_rate(asset, env_ids, rate: torch.Tensor, quat_w: torch.Tensor, base_vel: torch.Tensor | None = None):
    """Set the base rotating at ``rate`` [rad/s] about X through the foot tip (added to ``base_vel``, link frame)."""
    omega = torch.zeros(rate.shape[0], 3, device=rate.device)
    omega[:, 0] = rate
    vel = torch.cat([torch.cross(omega, _tip_to_origin_w(quat_w), dim=-1), omega], dim=-1)
    if base_vel is not None:
        vel += base_vel
    asset.write_root_link_velocity_to_sim_index(root_velocity=vel, env_ids=env_ids)


def reset_tilted(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | slice,
    tilt_range: tuple[float, float],
    tilt_rate_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Stand the base on its foot tip (at the env origin) tilted about X and rotating about the tip."""
    asset = env.scene[asset_cfg.name]
    n = asset.data.default_root_pose.torch[env_ids].shape[0]
    tilt = math_utils.sample_uniform(*tilt_range, (n,), device=asset.device)
    quat = math_utils.quat_from_euler_xyz(tilt, torch.zeros_like(tilt), torch.zeros_like(tilt))
    pos = env.scene.env_origins[env_ids] + _tip_to_origin_w(quat)
    asset.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], dim=-1), env_ids=env_ids)
    rate = math_utils.sample_uniform(*tilt_rate_range, (n,), device=asset.device)
    _write_tilt_rate(asset, env_ids, rate, quat)


def tilt_x(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Tilt about X [rad] from the IMU gravity: the only way the base can fall.

    Not the full gravity vector: its X part (tilt about Y) stays ~0 on the foot line, so the observation normalizer
    learned a std of 1e-4 there, and the sub-mm rocking PhysX (or a real IMU) shows was then several sigma off and
    threw the policy (a Newton policy fell on PhysX within 0.1 s).
    """
    g = env.scene[asset_cfg.name].data.projected_gravity_b.torch
    return torch.atan2(-g[:, 1], -g[:, 2]).unsqueeze(-1)


def tilt_rate_x(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Gyro rate about X [rad/s] (the Y / Z rates are ~0 likewise)."""
    return env.scene[asset_cfg.name].data.root_ang_vel_b.torch[:, :1]


def upright(env: ManagerBasedEnv, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """exp(-(tilt / std)^2): 1 upright."""
    tilt = torch.acos((-env.scene[asset_cfg.name].data.projected_gravity_b.torch[:, 2]).clamp(-1.0, 1.0))
    return torch.exp(-((tilt / std) ** 2))


def still(env: ManagerBasedEnv, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """exp(-(tilt rate / std)^2): 1 with the base at rest."""
    rate = env.scene[asset_cfg.name].data.root_ang_vel_b.torch[:, :2].norm(dim=-1)
    return torch.exp(-((rate / std) ** 2))


def log_wheel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = WHEEL) -> torch.Tensor:
    """Never terminates: logs the wheel torque (after the DC motor limit) and speed to Metrics/ (TensorBoard)."""
    asset = env.scene[asset_cfg.name]
    torque = asset.actuators.applied_effort.torch[:, asset_cfg.joint_ids].abs()
    speed = asset.data.joint_vel.torch[:, asset_cfg.joint_ids].abs()
    log = env.extras.setdefault("log", {})
    log["Metrics/wheel_torque_Nm"] = torque.mean()
    log["Metrics/wheel_speed_rad_s"] = speed.mean()
    log["Metrics/wheel_speed_max_rad_s"] = speed.max()
    log["Metrics/wheel_saturated_rate"] = (speed > 0.9 * WHEEL_NO_LOAD_SPEED).float().mean()
    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)


def push_tilt(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | slice,
    tilt_rate_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Push: add a random tilt rate about the foot tip to the current base motion."""
    asset = env.scene[asset_cfg.name]
    quat = asset.data.root_link_quat_w.torch[env_ids]
    rate = math_utils.sample_uniform(*tilt_rate_range, (quat.shape[0],), device=asset.device)
    _write_tilt_rate(asset, env_ids, rate, quat, asset.data.root_link_vel_w.torch[env_ids])


@configclass
class ActionsCfg:
    # wheel torque, [-1, 1] -> [-ACTION_TORQUE, ACTION_TORQUE] N m (the DC motor model clips it further with speed)
    wheel = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["wheel"], scale=ACTION_TORQUE)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # IMU noise in training only (play_mode turns corruption off): keeps the normalizer and the policy off
        # noise-free values a real sensor never gives
        tilt = ObsTerm(func=tilt_x, noise=Gnoise(std=0.002))
        tilt_rate = ObsTerm(func=tilt_rate_x, noise=Gnoise(std=0.01))
        wheel_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": WHEEL}, scale=1.0 / WHEEL_NO_LOAD_SPEED)
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    # domain randomization, once per env at startup (Newton does not fully update the mass matrix after a runtime
    # CoM change): the base CoM by +-5 mm about the CAD value, and the foot friction in [0.5, 1] (the ground's 0.5
    # combines by max, balance_env_cfg.py, so the foot value acts)
    base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["base"]),
            "com_range": {"x": (-0.01, 0.001), "y": (-0.010, 0.010), "z": (-0.010, 0.010)},
        },
    )
    foot_friction = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["base"]),
            "static_friction_range": (0.5, 1.0),
            "dynamic_friction_range": (0.5, 1.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,
        },
    )
    reset_base = EventTerm(
        # well within what the wheel can recover (about 0.37 rad at rest, robots.py)
        func=reset_tilted, mode="reset", params={"tilt_range": (-0.15, 0.15), "tilt_rate_range": (-0.35, 0.35)}
    )
    reset_wheel = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={"asset_cfg": WHEEL, "position_range": (-3.14, 3.14), "velocity_range": (-5.0, 5.0)},
    )
    # up to ~1.2e-3 N m s about the tip, a tenth of what the wheel can take up
    push = EventTerm(
        func=push_tilt, mode="interval", interval_range_s=(1.0, 3.0), params={"tilt_rate_range": (-0.5, 0.5)}
    )


@configclass
class RewardCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    fallen = RewTerm(func=mdp.is_terminated, weight=-100.0)
    upright = RewTerm(func=upright, weight=0.1, params={"std": 0.05})
    # the base at rest, not swaying about upright
    still = RewTerm(func=still, weight=1.0, params={"std": 0.2})
    wheel_speed = RewTerm(func=mdp.joint_vel_l2, weight=-1.0e-6, params={"asset_cfg": WHEEL})
    torque = RewTerm(func=mdp.action_l2, weight=-1.0e-7)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    fallen = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": FALL_TILT})
    wheel_metrics = DoneTerm(func=log_wheel, params={"asset_cfg": WHEEL})
