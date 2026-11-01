"""MDP of lifting the red box with the CRX-5iA + 2F-85: the term functions and the action / observation / reward /
termination configs (the scene and the env are in lift_env_cfg.py).

Shaped after Isaac Lab's SO-101 vial tutorial (IsaacLabTutorial, place_vial): small incremental joint actions, a
potential-based approach term, a once-per-episode grasp milestone, a large success bonus that ends the episode, and a
penalty for losing the box. Nothing pays a large amount per step for merely holding the box, so finishing the task
is what pays.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs import mdp
from isaaclab.envs.mdp.actions import BinaryJointPositionActionCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ManagerTermBase
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.math import matrix_from_quat, subtract_frame_transforms

from fanuc_sim.objects import RED_BOX_SIZE
from sim_base.actions import IncrementalJointPositionAction, IncrementalJointPositionActionCfg  # noqa: F401

if TYPE_CHECKING:
    # importing the env class pulls in pxr, which must not load before Kit starts (the RL CLI reads this cfg first)
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import RewardTermCfg, TerminationTermCfg

ARM = ["J[1-6]"]
ARM_AND_FINGER = ["J[1-6]", "finger_joint"]
FINGERTIPS = ["left_fingertip", "right_fingertip"]
RED_BOX = SceneEntityCfg("red_box")
# red box center height when it rests on the ground
RED_REST_HEIGHT = RED_BOX_SIZE[2] / 2
# success: the grasped box held this far above its rest height [m] for SUCCESS_STEPS control steps
LIFT_HEIGHT = 0.10
SUCCESS_STEPS = 10
# "lifted off" milestone (a critic observation) [m]
LIFTED_OFF = 0.04
# held: after the grasp milestone, the box center stays this close to the tcp [m]. Not the contact forces: they
# flicker on and off every few steps while the box is squeezed, which broke every "N steps in a row" check.
HELD_MAX_TCP_DISTANCE = 0.04
# gripper body whose frame is the tcp orientation (the tcp is fixed on it)
GRIPPER_BASE = "base_link"
# robot links watched for floor contact (one contact sensor each: PhysX filters contacts per single body)
GRIPPER_LINKS = [
    "base_link",
    *(f"{side}_{link}" for side in ("left", "right")
      for link in ("outer_knuckle", "outer_finger", "inner_finger", "inner_knuckle", "fingertip")),
]
FLOOR_CONTACT_LINKS = {"J6_link": "J6_link", **{n: f"J6_link/Robotiq_2F_85/Robotiq_2F_85/{n}" for n in GRIPPER_LINKS}}
"""Link name -> prim path under the robot."""
FLOOR_COLLIDER = "/World/GroundPlane/GroundPlane/CollisionPlane"
# finger links watched for contact with the red box (grasp detection): the fingertip pads hold the box
GRASP_LINKS = {n: f"J6_link/Robotiq_2F_85/Robotiq_2F_85/{n}" for n in FINGERTIPS}
RED_BOX_PRIM = "{ENV_REGEX_NS}/RedBox"
# grasp check: finger_joint stops at 0.29 rad on the 50 mm faces, 0.09 rad across the diagonal, 0.8 rad on nothing
GRASP_MAX_FINGER = 0.6
# grasp check: cosine between the two pad forces; squeezing pushes the pads apart (-1), pressing from above does not
GRASP_MAX_PAD_COS = -0.5
# farthest red box (horizontal, from the robot base) the gripper can still take from above [m]
RED_BOX_MAX_REACH = 0.8
# red box spawn range around its default position (objects.py) [m]
RED_BOX_SPAWN_RANGE = {"x": (-0.1, 0.1), "y": (-0.1, 0.1)}

# ---- actions -------------------------------------------------------------------------------------------------------


# IncrementalJointPositionAction(Cfg): sim_base.actions (shared with the other robots' tasks)


# ---- task progress -------------------------------------------------------------------------------------------------


def _height(env: ManagerBasedRLEnv, object_cfg: SceneEntityCfg) -> torch.Tensor:
    """Object center height above the env origin (the ground)."""
    return env.scene[object_cfg.name].data.root_pos_w.torch[:, 2] - env.scene.env_origins[:, 2]


def _grasped(env: ManagerBasedRLEnv, threshold: float = 1.0) -> torch.Tensor:
    """The red box is squeezed between the fingertip pads: both press on it with more than ``threshold`` [N], from
    opposite sides, and the fingers stopped on it short of fully closed.

    Without the last two, a closed hand pushed onto the box (both pads on its top face, forces parallel) counts too.
    """
    pads = [env.scene.sensors[f"{name}_red_box"].data.normal_force_matrix_w.torch[:, 0, 0] for name in GRASP_LINKS]
    left, right = (torch.norm(f, dim=-1) for f in pads)
    cos = (pads[0] * pads[1]).sum(dim=-1) / (left * right).clamp(min=1e-6)
    robot = env.scene["robot"]
    finger = robot.data.joint_pos.torch[:, robot.joint_names.index("finger_joint")]
    return (left > threshold) & (right > threshold) & (cos < GRASP_MAX_PAD_COS) & (finger < GRASP_MAX_FINGER)


class LiftProgress:
    """Episode-local milestones, latched the first time they hold (as the tutorial's PlacementProgress):

    * ``grasped``: both pads on the box for ``grasp_steps`` consecutive steps,
    * ``lifted``: the held box ever above its rest height by LIFTED_OFF,
    * ``success``: the held box above its rest height by LIFT_HEIGHT for SUCCESS_STEPS consecutive steps.

    ``held`` (not latched) is updated each step as well: see :func:`_held`.
    """

    def __init__(self, num_envs: int, device, grasp_steps: int = 2):
        self.grasp_steps = grasp_steps
        self.grasped = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.lifted = torch.zeros_like(self.grasped)
        self.success = torch.zeros_like(self.grasped)
        self.held = torch.zeros_like(self.grasped)
        self.grasp_count = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.high_count = torch.zeros_like(self.grasp_count)

    def reset(self, env_ids=None) -> None:
        ids = slice(None) if env_ids is None else env_ids
        for buf in (self.grasped, self.lifted, self.success, self.held):
            buf[ids] = False
        self.grasp_count[ids] = 0
        self.high_count[ids] = 0

    def update(self, env: ManagerBasedRLEnv) -> None:
        squeezed = _grasped(env)
        self.grasp_count = torch.where(squeezed, self.grasp_count + 1, torch.zeros_like(self.grasp_count))
        self.grasped |= self.grasp_count >= self.grasp_steps
        near = torch.norm(tcp_pos_w(env) - env.scene[RED_BOX.name].data.root_pos_w.torch, dim=1) < HELD_MAX_TCP_DISTANCE
        self.held = self.grasped & near
        height = _height(env, RED_BOX) - RED_REST_HEIGHT
        self.lifted |= self.held & (height > LIFTED_OFF)
        high = self.held & (height > LIFT_HEIGHT)
        self.high_count = torch.where(high, self.high_count + 1, torch.zeros_like(self.high_count))
        self.success |= self.high_count >= SUCCESS_STEPS


def _progress(env: ManagerBasedRLEnv) -> LiftProgress:
    if not hasattr(env, "_lift_progress"):
        env._lift_progress = LiftProgress(env.num_envs, env.device)
    return env._lift_progress


def _held(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The box is carried: grasped once this episode and still at the tcp. A box this high and this close to the
    fingers moves with them; no contact force needed."""
    return _progress(env).held


class LiftSuccessTerm(ManagerTermBase):
    """Termination that also advances the milestones each step (terminations are computed before rewards)."""

    def __init__(self, cfg: TerminationTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        _progress(self._env).reset(env_ids)

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        progress = _progress(env)
        progress.update(env)
        return progress.success.clone()


# ---- observations --------------------------------------------------------------------------------------------------


def _to_root_frame(env: ManagerBasedRLEnv, pos_w: torch.Tensor, robot_cfg: SceneEntityCfg) -> torch.Tensor:
    robot = env.scene[robot_cfg.name]
    pos_b, _ = subtract_frame_transforms(robot.data.root_pos_w.torch, robot.data.root_quat_w.torch, pos_w)
    return pos_b


def tcp_pos_w(env: ManagerBasedRLEnv, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Grasp point: midpoint of the two fingertip CoMs (the finger link frames all sit at the gripper base)."""
    robot = env.scene[robot_cfg.name]
    ids, _ = robot.find_bodies(FINGERTIPS, preserve_order=True)
    return robot.data.body_com_pos_w.torch[:, ids, :].mean(dim=1)


def tcp_pos_b(env: ManagerBasedRLEnv, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Grasp point in the robot root frame."""
    return _to_root_frame(env, tcp_pos_w(env, robot_cfg), robot_cfg)


def object_pos_b(
    env: ManagerBasedRLEnv, object_cfg: SceneEntityCfg, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Object position in the robot root frame."""
    return _to_root_frame(env, env.scene[object_cfg.name].data.root_pos_w.torch, robot_cfg)


def object_quat_b(
    env: ManagerBasedRLEnv, object_cfg: SceneEntityCfg, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Object orientation in the robot root frame (quaternion x, y, z, w)."""
    robot, obj = env.scene[robot_cfg.name], env.scene[object_cfg.name]
    _, quat_b = subtract_frame_transforms(
        robot.data.root_pos_w.torch, robot.data.root_quat_w.torch, obj.data.root_pos_w.torch, obj.data.root_quat_w.torch
    )
    return quat_b


def object_lin_vel_w(env: ManagerBasedRLEnv, object_cfg: SceneEntityCfg) -> torch.Tensor:
    return env.scene[object_cfg.name].data.root_lin_vel_w.torch


def arm_target_rel(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The incremental arm action's joint targets relative to the default pose: they are hidden state otherwise."""
    term = env.action_manager.get_term("arm_action")
    robot = env.scene["robot"]
    return term.target - robot.data.default_joint_pos.torch[:, term._joint_ids]


def progress_flags(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Latched milestones, so the once-per-episode grasp reward stays Markov."""
    progress = _progress(env)
    return torch.stack((progress.grasped, progress.lifted), dim=-1).float()


# ---- rewards -------------------------------------------------------------------------------------------------------


class ApproachProgressReward(ManagerTermBase):
    """Pay the decrease of the tcp to red box distance, in units of ``scale`` [m] and clipped to +-1, until grasped.

    A difference of potentials: hovering pays nothing, and the gradient is the same far from the box as near it.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._previous = torch.zeros(self.num_envs, device=self.device)
        self._has_previous = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._has_previous[slice(None) if env_ids is None else env_ids] = False

    def __call__(self, env: ManagerBasedRLEnv, scale: float = 0.01) -> torch.Tensor:
        distance = torch.norm(tcp_pos_w(env) - env.scene[RED_BOX.name].data.root_pos_w.torch, dim=1)
        progress = ((self._previous - distance) / scale).clamp(-1.0, 1.0)
        active = self._has_previous & ~_progress(env).grasped
        self._previous.copy_(distance)
        self._has_previous.fill_(True)
        return torch.where(active, progress, torch.zeros_like(progress))

class LiftProgressReward(ManagerTermBase):
    """While the box is held: pay each new best height of the episode, in units of ``scale`` [m] and clipped to 1, up
    to the success height. The lift counterpart of ApproachProgressReward.

    Only new bests pay, so moving the box up and down earns nothing: with a plain height difference, every rise paid
    while the drops that came between were free whenever the hold check missed a step.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._best = torch.full((self.num_envs,), RED_REST_HEIGHT, device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._best[slice(None) if env_ids is None else env_ids] = RED_REST_HEIGHT

    def __call__(self, env: ManagerBasedRLEnv, scale: float = 0.01) -> torch.Tensor:
        # 箱の高さ（成功の高さより上は数えない）
        height = _height(env, RED_BOX).clamp(max=RED_REST_HEIGHT + LIFT_HEIGHT)
        held = _held(env)
        # このエピソードの最高到達点を更新した分だけ、1 cm で 1、1 で頭打ち
        gain = ((height - self._best) / scale).clamp(0.0, 1.0)
        self._best = torch.where(held, torch.maximum(self._best, height), self._best)
        return torch.where(held, gain, torch.zeros_like(gain))


class GraspMilestoneReward(ManagerTermBase):
    """1 on the step the grasp milestone is first reached in an episode."""

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._previous = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._previous[slice(None) if env_ids is None else env_ids] = False

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        current = _progress(env).grasped
        reward = (current & ~self._previous).float()
        self._previous.copy_(current)
        return reward


def held_lift_reward(env: ManagerBasedRLEnv, std: float = 0.05) -> torch.Tensor:
    """While the box is held: exp(-(height still to go) / std), small dense shaping toward the success height."""
    to_go = torch.clamp(RED_REST_HEIGHT + LIFT_HEIGHT - _height(env, RED_BOX), min=0.0)
    return _held(env).float() * torch.exp(-to_go / std)


def tcp_box_angle(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Relative angle between the tcp frame and the red box [rad], 0 (square) .. 0.96 (diagonal).

    The box is a cube, so each tcp axis is taken to its nearest box axis, and the worst of the three counts.
    """
    robot, box = env.scene["robot"], env.scene[RED_BOX.name]
    base, _ = robot.find_bodies([GRIPPER_BASE])
    tcp_quat = robot.data.body_quat_w.torch[:, base[0]]
    _, rel_quat = subtract_frame_transforms(
        tcp_pos_w(env), tcp_quat, box.data.root_pos_w.torch, box.data.root_quat_w.torch
    )
    # rel[:, k, j]: cosine between tcp axis k and box axis j
    nearest = matrix_from_quat(rel_quat).abs().amax(dim=-1)
    return torch.acos(nearest.clamp(max=1.0)).amax(dim=-1)


class AlignmentProgressReward(ManagerTermBase):
    """Pay the decrease of the tcp to box angle, in units of ``scale`` [rad] and clipped to +-1, until grasped.

    Squaring the hand to the box faces before closing gives the face grasp (finger_joint 0.29 rad) rather than the
    weaker diagonal one (0.09 rad). A difference of potentials, as ApproachProgressReward: a reward per step for
    being aligned paid more for hovering square to the box than the grasp bonus did, and a run settled there.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._previous = torch.zeros(self.num_envs, device=self.device)
        self._has_previous = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._has_previous[slice(None) if env_ids is None else env_ids] = False

    def __call__(self, env: ManagerBasedRLEnv, scale: float = 0.1) -> torch.Tensor:
        angle = tcp_box_angle(env)
        progress = ((self._previous - angle) / scale).clamp(-1.0, 1.0)
        active = self._has_previous & ~_progress(env).grasped
        self._previous.copy_(angle)
        self._has_previous.fill_(True)
        return torch.where(active, progress, torch.zeros_like(progress))


def success_bonus(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _progress(env).success.float()


def red_box_lost(env: ManagerBasedRLEnv, max_distance: float = RED_BOX_MAX_REACH) -> torch.Tensor:
    """Out of reach, or non-finite.

    Dropping is not lost: a drop penalty armed only once the box was lifted off taught the policy to stop just
    below that height. A drop already costs the held_lift reward and the success bonus.
    """
    red = env.scene[RED_BOX.name]
    finite = torch.isfinite(red.data.root_pos_w.torch).all(dim=-1)
    far = torch.norm(object_pos_b(env, RED_BOX)[:, :2], dim=1) > max_distance
    return (~finite) | far


def unstable_robot(env: ManagerBasedRLEnv, max_joint_vel: float = 12.0) -> torch.Tensor:
    vel = env.scene["robot"].data.joint_vel.torch
    return (~torch.isfinite(vel).all(dim=-1)) | (vel.abs().amax(dim=1) > max_joint_vel)


def floor_contact(env: ManagerBasedRLEnv, threshold: float = 1.0) -> torch.Tensor:
    """Number of watched robot links pushing on the floor with more than ``threshold`` [N]."""
    count = torch.zeros(env.num_envs, device=env.device)
    for name in FLOOR_CONTACT_LINKS:
        forces = env.scene.sensors[f"{name}_floor"].data.normal_force_matrix_w.torch  # (envs, 1, 1, 3)
        count += (torch.norm(forces, dim=-1).amax(dim=(1, 2)) > threshold).float()
    return count


# ---- configs -------------------------------------------------------------------------------------------------------

@configclass
class ActionsCfg:
    # 0.02 rad per physics step (decimation 2: 0.04 per 50 Hz control step, at most ~2 rad/s). The tutorial found
    # larger steps raised the failure rate.
    arm_action = IncrementalJointPositionActionCfg(asset_name="robot", joint_names=ARM, scale=0.02)
    # 2F-85: 1 action, < 0 closes, >= 0 opens. Only finger_joint is driven; the other finger joints follow it
    # through the asset's linkage. 0.8 rad = fully closed without an object (check_crx5ia_2f85.py).
    gripper_action = BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["finger_joint"],
        open_command_expr={"finger_joint": 0.0},
        close_command_expr={"finger_joint": 0.8},
    )

@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(ObsGroup):
        # arm + driven finger joint (the passive finger joints follow finger_joint)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_AND_FINGER)})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_AND_FINGER)})
        arm_target = ObsTerm(func=arm_target_rel)
        # positions in the robot root frame [m]
        tcp_pos = ObsTerm(func=tcp_pos_b)
        red_box_pos = ObsTerm(func=object_pos_b, params={"object_cfg": RED_BOX})
        red_box_quat = ObsTerm(func=object_quat_b, params={"object_cfg": RED_BOX})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Privileged terms for the critic only (on top of the policy group): not available on the real robot."""

        red_box_vel = ObsTerm(func=object_lin_vel_w, params={"object_cfg": RED_BOX})
        # grasped / lifted, judged from the fingertip contact forces
        progress = ObsTerm(func=progress_flags)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()

@configclass
class EventCfg:
    # 起動時に 1 回、環境ごとに箱の摩擦をばらつかせる
    red_box_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": RED_BOX,
            "static_friction_range": (0.5, 1.2),
            "dynamic_friction_range": (0.4, 1.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,   # 動摩擦 ≤ 静止摩擦 に揃える
        },
    )
    # 同じく指先パッドの摩擦（PhysX は箱とパッドの平均を使うので、両方ばらつかせる）
    fingertip_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FINGERTIPS),
            "static_friction_range": (0.6, 1.2),
            "dynamic_friction_range": (0.5, 1.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,
        },
    )
    # scene to its default state first (overriding the events drops Isaac Lab's default reset), then move the red box
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")
    reset_red_box = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": RED_BOX_SPAWN_RANGE, "velocity_range": {}, "asset_cfg": RED_BOX},
    )

@configclass
class RewardCfg:
    # dense: distance closed toward the box, and while holding, nearness to the success height
    approach = RewTerm(func=ApproachProgressReward, weight=1.0)
    lift = RewTerm(func=LiftProgressReward, weight=1.0)
    held_lift = RewTerm(func=held_lift_reward, weight=0.1)
    # turning the tcp square to the box faces before the grasp (0.1 rad of angle closed = 1)
    alignment = RewTerm(func=AlignmentProgressReward, weight=0.1)
    # sparse: the grasp once, success once (ends the episode), losing the box (ends it too)
    grasp = RewTerm(func=GraspMilestoneReward, weight=10.0)
    success = RewTerm(func=success_bonus, weight=200.0)
    lost = RewTerm(func=red_box_lost, weight=-50.0)
    # per robot link touching the floor
    floor_contact = RewTerm(func=floor_contact, weight=-0.2)
    # regularization (the tutorial's weights)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.002)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.0002, params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM)})

@configclass
class TerminationsCfg:
    # first: it advances the milestones the rewards read
    success = DoneTerm(func=LiftSuccessTerm)
    lost = DoneTerm(func=red_box_lost)
    unstable_robot = DoneTerm(func=unstable_robot)
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
