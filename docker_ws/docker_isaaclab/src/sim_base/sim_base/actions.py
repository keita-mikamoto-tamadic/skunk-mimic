"""Joint actions for the RL tasks of the isaaclab_ws robots."""

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.envs.mdp.actions import DifferentialInverseKinematicsActionCfg, JointPositionAction, JointPositionActionCfg
from isaaclab.utils import configclass
from isaaclab_newton.envs.mdp.actions.newton_ik_actions_cfg import NewtonInverseKinematicsActionCfg


class IncrementalJointPositionAction(JointPositionAction):
    """Joint targets moved by a bounded increment per physics step: target += scale * clip(action, -1, 1).

    The increment is applied on every physics step (``apply_actions``), so ``decimation`` times per policy step, as a
    step change. Kept so for the policies trained with it (CRX lift / place); :class:`InterpolatedJointPositionAction`
    moves by ``scale`` per policy step, interpolated over its physics steps.

    The target is kept between steps rather than re-based on the measured position (Isaac Lab's relative action):
    re-basing gives the drive no error to hold against gravity, so an arm would sag at zero action. The target stays
    within ``max_lead`` [rad] of the measured position, so it cannot wind up while the arm is blocked.
    """

    cfg: IncrementalJointPositionActionCfg

    def __init__(self, cfg: IncrementalJointPositionActionCfg, env):
        super().__init__(cfg, env)
        self._target = self._asset.data.default_joint_pos.torch[:, self._joint_ids].clone()

    @property
    def target(self) -> torch.Tensor:
        return self._target

    def process_actions(self, actions: torch.Tensor) -> None:
        super().process_actions(torch.nan_to_num(actions, nan=0.0, posinf=1.0, neginf=-1.0).clamp(-1.0, 1.0))

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._target[ids] = self._asset.data.joint_pos.torch[ids][:, self._joint_ids]
        # a reset event that loads a recorded state (e.g. the place task) also hands over the target it was held with:
        # the measured position alone sags under gravity
        seed = getattr(self._env, "_reset_arm_target", None)
        if seed is not None:
            use = self._env._use_reset_arm_target[ids].unsqueeze(-1)
            self._target[ids] = torch.where(use, seed[ids], self._target[ids])

    def apply_actions(self) -> None:
        measured = self._asset.data.joint_pos.torch[:, self._joint_ids]
        lead = self.cfg.max_lead
        self._target = torch.clamp(self._target + self.processed_actions, measured - lead, measured + lead)
        limits = self._asset.data.soft_joint_pos_limits.torch[:, self._joint_ids]
        self._target = torch.clamp(self._target, limits[..., 0], limits[..., 1])
        self._asset.set_joint_position_target_index(target=self._target, joint_ids=self._joint_ids)


@configclass
class IncrementalJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = IncrementalJointPositionAction
    use_default_offset: bool = False
    max_lead: float = 0.1
    """Largest gap between the target and the measured joint position [rad]."""


class InterpolatedJointPositionAction(IncrementalJointPositionAction):
    """Joint targets moved by ``scale * clip(action, -1, 1)`` per policy step, linearly over its physics steps.

    The policy gives the end point of the step; the start point is where the previous step ended. Each physics step
    (``decimation`` per policy step) gets the next point of the line, so the drive sees no step change.
    """

    cfg: InterpolatedJointPositionActionCfg

    def __init__(self, cfg: InterpolatedJointPositionActionCfg, env):
        super().__init__(cfg, env)
        self._start = self._target.clone()
        self._end = self._target.clone()
        self._substep = 0

    def process_actions(self, actions: torch.Tensor) -> None:
        super().process_actions(actions)
        self._start = self._target.clone()
        self._end = self._target + self.processed_actions
        self._substep = 0

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._start[ids] = self._target[ids]
        self._end[ids] = self._target[ids]

    def apply_actions(self) -> None:
        self._substep += 1
        alpha = min(self._substep / self._env.cfg.decimation, 1.0)
        target = self._start + (self._end - self._start) * alpha
        measured = self._asset.data.joint_pos.torch[:, self._joint_ids]
        lead = self.cfg.max_lead
        self._target = torch.clamp(target, measured - lead, measured + lead)
        limits = self._asset.data.soft_joint_pos_limits.torch[:, self._joint_ids]
        self._target = torch.clamp(self._target, limits[..., 0], limits[..., 1])
        self._asset.set_joint_position_target_index(target=self._target, joint_ids=self._joint_ids)


@configclass
class InterpolatedJointPositionActionCfg(IncrementalJointPositionActionCfg):
    class_type: type = InterpolatedJointPositionAction


@configclass
class InterpolatedNewtonIKActionCfg(NewtonInverseKinematicsActionCfg):
    """Isaac Lab's Newton IK action with each policy step's pose targets interpolated over its physics steps
    (:class:`~sim_base.newton_ik_action.InterpolatedNewtonIKAction`). Needs Newton physics, as the base."""

    # a string: the action module imports Newton / pxr, which must not load before Kit (the RL CLI reads cfgs first)
    class_type: type | str = "sim_base.newton_ik_action:InterpolatedNewtonIKAction"
    max_goal_lead: float = 0.03
    """Largest gap between a position goal and the measured point [m]."""


@configclass
class InterpolatedDifferentialIKActionCfg(DifferentialInverseKinematicsActionCfg):
    """Isaac Lab's differential IK action with the tcp goal kept between policy steps and interpolated over its
    physics steps (:class:`~sim_base.diff_ik_action.InterpolatedDifferentialIKAction`). ``scale``: largest goal move
    per policy step [m]; the controller must take an absolute position command (``use_relative_mode=False``).

    The policy's action is still an increment: tools that read the controller cfg as an absolute command (Isaac Lab's
    zero agent sends the current tcp as a "hold" action) misjudge it; zero actions hold here."""

    class_type: type | str = "sim_base.diff_ik_action:InterpolatedDifferentialIKAction"
    max_goal_lead: float = 0.03
    """Largest gap between the goal and the measured tcp [m]."""
