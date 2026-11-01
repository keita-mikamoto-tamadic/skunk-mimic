"""MDP of placing the red box on the fixed yellow box with the CRX-5iA + 2F-85 (the scene and the env are in
place_env_cfg.py).

Builds on the lift task (tasks/lift/mdp.py): its actions, grasp check, held check, approach and alignment terms are
used as they are. Shaped after the SO-101 vial tutorial's placement (IsaacLabTutorial, place_vial): physical
milestones paid once each, a dense pull toward the goal only while the box is carried and not yet over the target,
and a success that needs the box released and resting, so letting go is the only profitable continuation.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs import mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ManagerTermBase
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.math import matrix_from_quat

from fanuc_sim.objects import RED_BOX_SIZE, YELLOW_BOX_SIZE
from fanuc_sim.tasks.lift import mdp as lift
from fanuc_sim.tasks.lift.mdp import (
    ARM,
    HELD_MAX_TCP_DISTANCE,
    RED_BOX,
    RED_REST_HEIGHT,
    AlignmentProgressReward,
    ApproachProgressReward,
    LiftProgress,
)
from fanuc_sim.tasks.place.reset_states import ResetFromStates

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import TerminationTermCfg

YELLOW_BOX = SceneEntityCfg("yellow_box")
YELLOW_BOX_PRIM = "{ENV_REGEX_NS}/YellowBox"
# robot links watched for contact with the yellow box (one contact sensor each, scene names <link>_yellow_box):
# the arm links too, the box stands close to the base. Link name -> prim path under the robot.
YELLOW_CONTACT_LINKS = {**{f"J{i}_link": f"J{i}_link" for i in range(1, 6)}, **lift.FLOOR_CONTACT_LINKS}
RED_HALF = RED_BOX_SIZE[2] / 2
YELLOW_HALF = YELLOW_BOX_SIZE[2] / 2
# "lifted": the held box's bottom this far above the yellow top [m]
LIFT_CLEARANCE = 0.02
# "over": held with its bottom at most this far above the yellow top [m], and its center within OVER_RADIUS of the
# yellow top's center (horizontal) [m]: the box only has to be let go
OVER_MAX_GAP = 0.02
OVER_RADIUS = 0.04
# goal of the carried box: its center above the yellow top's center, the bottom this far above the top [m]
GOAL_GAP = 0.01
# placed: resting on the yellow top, center within PLACED_RADIUS (horizontal) [m], bottom within PLACED_MAX_GAP of
# the top [m], upright, still, and released
PLACED_RADIUS = YELLOW_BOX_SIZE[0] / 2 - RED_BOX_SIZE[0] / 2
PLACED_MAX_GAP = 0.01
UPRIGHT_MIN_COS = 0.95
STILL_MAX_SPEED = 0.05
# released: finger_joint opened past the 50 mm box (it stops at 0.29 rad on it)
RELEASE_MAX_FINGER = 0.15
SUCCESS_STEPS = 10
# start phase weights (reset_states.PHASES: home, pregrasp, grasped, lifted): every phase from the first iteration
# in training; play and evaluation start from home
TRAIN_PHASE_WEIGHTS = (1.0, 1.0, 1.0, 1.0)
PLAY_PHASE_WEIGHTS = (1.0, 0.0, 0.0, 0.0)

# ---- geometry ------------------------------------------------------------------------------------------------------


def _yellow_top_w(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Center of the yellow box's top face."""
    top = env.scene[YELLOW_BOX.name].data.root_pos_w.torch.clone()
    top[:, 2] += YELLOW_HALF
    return top


def goal_pos_w(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Where the carried red box's center should go: just above the middle of the yellow top."""
    goal = _yellow_top_w(env)
    goal[:, 2] += RED_HALF + GOAL_GAP
    return goal


def _placement(env: ManagerBasedRLEnv) -> tuple[torch.Tensor, torch.Tensor]:
    """Red box center to yellow top center: horizontal distance, and bottom gap above the top [m]."""
    red = env.scene[RED_BOX.name].data.root_pos_w.torch
    top = _yellow_top_w(env)
    return torch.norm(red[:, :2] - top[:, :2], dim=1), red[:, 2] - RED_HALF - top[:, 2]


def _finger(env: ManagerBasedRLEnv) -> torch.Tensor:
    robot = env.scene["robot"]
    return robot.data.joint_pos.torch[:, robot.joint_names.index("finger_joint")]


# ---- task progress -------------------------------------------------------------------------------------------------


class PlaceProgress(LiftProgress):
    """Episode-local milestones, latched the first time they hold:

    * ``grasped``: both pads squeeze the box for ``grasp_steps`` consecutive steps (as the lift task),
    * ``lifted``: the held box's bottom above the yellow top by LIFT_CLEARANCE,
    * ``over``: the held box low over the middle of the yellow top (OVER_RADIUS, OVER_MAX_GAP),
    * ``success``: the box placed (see :func:`_placed`) for SUCCESS_STEPS consecutive steps.

    ``held`` (not latched): grasped once and still at the tcp, as the lift task. It stands in for the lift task's
    progress (``env._lift_progress``), so the lift terms reused here (approach, alignment) read these milestones.
    """

    def __init__(self, num_envs: int, device, grasp_steps: int = 2):
        super().__init__(num_envs, device, grasp_steps)
        self.over = torch.zeros_like(self.grasped)
        self.place_count = torch.zeros_like(self.grasp_count)

    def reset(self, env_ids=None) -> None:
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self.over[ids] = False
        self.place_count[ids] = 0

    def update(self, env: ManagerBasedRLEnv) -> None:
        squeezed = lift._grasped(env)
        self.grasp_count = torch.where(squeezed, self.grasp_count + 1, torch.zeros_like(self.grasp_count))
        self.grasped |= self.grasp_count >= self.grasp_steps
        red = env.scene[RED_BOX.name].data.root_pos_w.torch
        self.held = self.grasped & (torch.norm(lift.tcp_pos_w(env) - red, dim=1) < HELD_MAX_TCP_DISTANCE)
        distance, gap = _placement(env)
        self.lifted |= self.held & (gap > LIFT_CLEARANCE)
        self.over |= self.held & (distance < OVER_RADIUS) & (gap > -0.005) & (gap < OVER_MAX_GAP)
        placed = _placed(env)
        self.place_count = torch.where(placed, self.place_count + 1, torch.zeros_like(self.place_count))
        self.success |= self.place_count >= SUCCESS_STEPS


def _placed(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The red box rests upright and still on the yellow top, and the fingers are open."""
    red = env.scene[RED_BOX.name]
    distance, gap = _placement(env)
    upright = matrix_from_quat(red.data.root_quat_w.torch)[:, 2, 2] > UPRIGHT_MIN_COS
    still = torch.norm(red.data.root_lin_vel_w.torch, dim=1) < STILL_MAX_SPEED
    released = _finger(env) < RELEASE_MAX_FINGER
    return (distance < PLACED_RADIUS) & (gap.abs() < PLACED_MAX_GAP) & upright & still & released


def _progress(env: ManagerBasedRLEnv) -> PlaceProgress:
    progress = getattr(env, "_lift_progress", None)
    if not isinstance(progress, PlaceProgress):
        progress = env._lift_progress = PlaceProgress(env.num_envs, env.device)
    return progress


class PlaceSuccessTerm(ManagerTermBase):
    """Termination that also advances the milestones each step (terminations are computed before rewards).

    Created with the managers, before any term runs, so the lift terms find the PlaceProgress in place.
    """

    def __init__(self, cfg: TerminationTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        _progress(env)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        progress = _progress(self._env)
        progress.reset(env_ids)
        # an env started from a recorded grasped state (reset_states.ResetFromStates) holds the box already
        seeded = getattr(self._env, "_reset_grasped", None)
        if seeded is not None:
            ids = slice(None) if env_ids is None else env_ids
            progress.grasped[ids] = seeded[ids]
            progress.grasp_count[ids] = torch.where(seeded[ids], progress.grasp_steps, 0)

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        progress = _progress(env)
        progress.update(env)
        log = env.extras.setdefault("log", {})
        for name in ("grasped", "lifted", "over", "success"):
            log[f"Metrics/{name}_rate"] = getattr(progress, name).float().mean()
        return progress.success.clone()


def red_box_dropped(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Back on the ground after it was lifted (dropped, or knocked off the yellow box): nothing left to do.

    A termination only, without a penalty: the lift task's drop penalty taught the policy to stop just below the
    height that armed it. Ending the episode already costs the success bonus.
    """
    height = env.scene[RED_BOX.name].data.root_pos_w.torch[:, 2] - env.scene.env_origins[:, 2]
    return _progress(env).lifted & (height < RED_REST_HEIGHT + 0.005)


def unstable_robot(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), max_joint_vel: float = 12.0
) -> torch.Tensor:
    """Non-finite or extreme velocity on the ``asset_cfg`` joints.

    The place task watches the arm and finger_joint only: the 2F-85's passive linkage joints rattle past 12 rad/s
    while the fingers squeeze the box, which cut 15-56% of the episodes, mostly while the box was carried.
    """
    vel = env.scene[asset_cfg.name].data.joint_vel.torch[:, asset_cfg.joint_ids]
    return (~torch.isfinite(vel).all(dim=-1)) | (vel.abs().amax(dim=1) > max_joint_vel)


# ---- observations --------------------------------------------------------------------------------------------------


def red_box_to_goal(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Goal minus red box center [m] (the robot base is not rotated, so world axes are its axes)."""
    return goal_pos_w(env) - env.scene[RED_BOX.name].data.root_pos_w.torch


def progress_flags(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Latched milestones (grasped, lifted, over), so the once-per-episode milestone reward stays Markov."""
    progress = _progress(env)
    return torch.stack((progress.grasped, progress.lifted, progress.over), dim=-1).float()


# ---- rewards -------------------------------------------------------------------------------------------------------


class MilestoneReward(ManagerTermBase):
    """Pay each milestone (grasped, lifted, over) once per episode, weighted 1 / 2 / 4.

    Milestones already set on the first step of an episode are not paid: a start from a recorded grasped state is
    not a grasp.
    """

    def __init__(self, cfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._previous = torch.zeros((self.num_envs, 3), dtype=torch.bool, device=self.device)
        self._started = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        ids = slice(None) if env_ids is None else env_ids
        self._previous[ids] = False
        self._started[ids] = False

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        current = progress_flags(env).bool()
        reward = ((current & ~self._previous).float() * current.new_tensor((1.0, 2.0, 4.0), dtype=torch.float)).sum(-1)
        reward = torch.where(self._started, reward, torch.zeros_like(reward))
        self._previous.copy_(current)
        self._started.fill_(True)
        return reward


class TransportProgressReward(ManagerTermBase):
    """While held and not yet over the target: pay each new closest approach of the red box to the goal, in units of
    ``scale`` [m] and clipped to 1.

    The carry counterpart of ApproachProgressReward, with the same gradient 36 cm from the goal as 2 cm away: the
    exp bump of :func:`transport_reward` alone is flat there, and the box was never lifted. Only new bests pay (as
    the lift task's lift term), so carrying the box back and forth earns nothing.
    """

    def __init__(self, cfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._best = torch.full((self.num_envs,), float("inf"), device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._best[slice(None) if env_ids is None else env_ids] = float("inf")

    def __call__(self, env: ManagerBasedRLEnv, scale: float = 0.01) -> torch.Tensor:
        progress = _progress(env)
        active = progress.held & ~progress.over
        distance = torch.norm(red_box_to_goal(env), dim=1)
        # the first held step only sets the best: inf - distance is not progress
        gain = torch.where(torch.isinf(self._best), torch.zeros_like(distance), (self._best - distance) / scale)
        self._best = torch.where(active, torch.minimum(self._best, distance), self._best)
        return torch.where(active, gain.clamp(0.0, 1.0), torch.zeros_like(gain))


def transport_reward(env: ManagerBasedRLEnv, std: float = 0.1) -> torch.Tensor:
    """While held and not yet over the target: exp(-(red box to goal distance) / std).

    Stops once over the target, so opening the fingers is what pays next. No such bump before the grasp: a policy
    that cannot grasp yet would hover in it (the tutorial's finding, and this project's alignment term).
    """
    progress = _progress(env)
    distance = torch.norm(red_box_to_goal(env), dim=1)
    return (progress.held & ~progress.over).float() * torch.exp(-distance / std)


def success_bonus(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _progress(env).success.float()


def yellow_box_contact(env: ManagerBasedRLEnv, threshold: float = 1.0) -> torch.Tensor:
    """Number of watched robot links pushing on the yellow box with more than ``threshold`` [N].

    The box is kinematic: it does not give way, so a policy could lean the arm on it, and on the real robot the arm
    or the box would break.
    """
    count = torch.zeros(env.num_envs, device=env.device)
    for name in YELLOW_CONTACT_LINKS:
        forces = env.scene.sensors[f"{name}_yellow_box"].data.normal_force_matrix_w.torch  # (envs, 1, 1, 3)
        count += (torch.norm(forces, dim=-1).amax(dim=(1, 2)) > threshold).float()
    return count


# ---- configs -------------------------------------------------------------------------------------------------------


@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(lift.ObservationsCfg.PolicyCfg):
        # positions in the robot root frame [m]
        yellow_box_pos = ObsTerm(func=lift.object_pos_b, params={"object_cfg": YELLOW_BOX})
        red_box_to_goal = ObsTerm(func=red_box_to_goal)

    @configclass
    class CriticCfg(ObsGroup):
        """Privileged terms for the critic only (on top of the policy group): not available on the real robot."""

        red_box_vel = ObsTerm(func=lift.object_lin_vel_w, params={"object_cfg": RED_BOX})
        # grasped / lifted / over, judged from the fingertip contact forces
        progress = ObsTerm(func=progress_flags)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg(lift.EventCfg):
    # last: overrides the default reset of the envs that start from a recorded phase (reset_states.py)
    reset_from_states = EventTerm(
        func=ResetFromStates, mode="reset", params={"phase_weights": TRAIN_PHASE_WEIGHTS}
    )


@configclass
class RewardCfg:
    # dense, until the grasp: distance closed toward the box, tcp squared to its faces (the lift task's terms)
    approach = RewTerm(func=ApproachProgressReward, weight=1.0)
    alignment = RewTerm(func=AlignmentProgressReward, weight=0.1)
    # dense, while carried and not yet over the target: closest approach to the goal (1 cm = 1), and near the goal
    carry = RewTerm(func=TransportProgressReward, weight=1.0)
    transport = RewTerm(func=transport_reward, weight=0.1)
    # sparse: grasped / lifted / over once each (10 / 20 / 40), success once (ends the episode)
    milestones = RewTerm(func=MilestoneReward, weight=10.0)
    success = RewTerm(func=success_bonus, weight=200.0)
    lost = RewTerm(func=lift.red_box_lost, weight=-50.0)
    # per robot link touching the floor, the yellow box
    floor_contact = RewTerm(func=lift.floor_contact, weight=-0.2)
    yellow_box_contact = RewTerm(func=yellow_box_contact, weight=-0.2)
    # regularization (the tutorial's weights)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.002)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.0002, params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM)})


@configclass
class TerminationsCfg:
    # first: it advances the milestones the rewards read
    success = DoneTerm(func=PlaceSuccessTerm)
    lost = DoneTerm(func=lift.red_box_lost)
    dropped = DoneTerm(func=red_box_dropped)
    unstable_robot = DoneTerm(
        func=unstable_robot, params={"asset_cfg": SceneEntityCfg("robot", joint_names=lift.ARM_AND_FINGER)}
    )
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
