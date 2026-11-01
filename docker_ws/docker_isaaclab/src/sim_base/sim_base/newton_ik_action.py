"""Isaac Lab's Newton IK action, interpolated over the physics steps of a policy step."""

from __future__ import annotations

from collections.abc import Sequence

import torch
import warp as wp

from isaaclab.utils.math import quat_apply, subtract_frame_transforms
from isaaclab_newton.controllers.ik.newton_ik_objectives_cfg import NewtonIKPoseObjectiveCfg
from isaaclab_newton.envs.mdp.actions.newton_ik_actions import NewtonInverseKinematicsAction


class InterpolatedNewtonIKAction(NewtonInverseKinematicsAction):
    """:class:`NewtonInverseKinematicsAction` (Newton IK, Levenberg-Marquardt on the GPU) for position objectives,
    with the goal kept between policy steps and interpolated over each step's physics steps.

    Goal: each policy step moves each pose objective's position goal by ``scale * clip(action, -1, 1)`` from where the
    previous step's goal was, kept within ``max_goal_lead`` [m] of the measured point. The base's relative mode
    re-bases on the measured point instead, so at zero action the IK answer is the measured joints, the drives have
    no error to hold against gravity and the arm sags (RSARM: 1.5 cm/s).
    Interpolation: the base solves IK to the targets on every physics step (``apply_actions``); here each physics step
    first writes the next point of the line from the previous goal into the targets the solver reads, so the joint
    targets follow a straight tcp path instead of jumping. Rotation targets are not driven (use rotation_weight 0).
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        # the solver's target arrays (allocated once), viewed as (num_envs, 3) tensors
        self._targets = [wp.to_torch(d.objective.position_objective.target_positions) for d in self._drivers]
        self._start = [t.clone() for t in self._targets]
        self._end = [t.clone() for t in self._targets]
        self._has_end = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._substep = 0
        pose_cfgs = [o for o in cfg.objectives if isinstance(o, NewtonIKPoseObjectiveCfg)]
        self._scales = [float(c.scale) for c in pose_cfgs]
        self._offsets = [torch.tensor(c.body_offset_pos, device=self.device).repeat(self.num_envs, 1) for c in pose_cfgs]

    @property
    def goal(self) -> torch.Tensor:
        """Position goal of the first pose objective at the end of this policy step, in the env's root frame."""
        return self._end[0]

    @property
    def has_goal(self) -> torch.Tensor:
        return self._has_end

    def _measured(self, i: int) -> torch.Tensor:
        """Pose objective i's point (body + offset) in the env's root frame."""
        data = self._asset.data
        body = self._drivers[i].body_idx
        point_w = data.body_pos_w.torch[:, body] + quat_apply(data.body_quat_w.torch[:, body], self._offsets[i])
        return subtract_frame_transforms(data.root_pos_w.torch, data.root_quat_w.torch, point_w)[0]

    def process_actions(self, actions: torch.Tensor) -> None:
        self._raw_actions[:] = actions
        self._processed_actions[:] = torch.nan_to_num(actions, nan=0.0, posinf=1.0, neginf=-1.0).clamp(-1.0, 1.0)
        has = self._has_end.unsqueeze(-1)
        lead = self.cfg.max_goal_lead
        for i, driver in enumerate(self._drivers):
            measured = self._measured(i)
            start = torch.where(has, self._end[i], measured)
            step = self._processed_actions[:, driver.action_offset : driver.action_offset + 3] * self._scales[i]
            self._start[i] = start
            self._end[i] = torch.clamp(start + step, measured - lead, measured + lead)
        self._has_end.fill_(True)
        self._substep = 0

    def apply_actions(self) -> None:
        self._substep += 1
        alpha = min(self._substep / self._env.cfg.decimation, 1.0)
        # the solver works in the env-0 prototype world: root-frame points go through env 0's root pose, as the
        # base's target kernel does
        root_pos, root_quat = self._asset.data.root_pos_w.torch[:1], self._asset.data.root_quat_w.torch[:1]
        for i, target in enumerate(self._targets):
            point = self._start[i] + (self._end[i] - self._start[i]) * alpha
            target.copy_(root_pos + quat_apply(root_quat.expand(self.num_envs, 4), point))
        super().apply_actions()

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        self._has_end[slice(None) if env_ids is None else env_ids] = False
