"""Isaac Lab's differential IK action (PhysX or Newton), interpolated over the physics steps of a policy step."""

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.envs.mdp.actions.task_space_actions import DifferentialInverseKinematicsAction


class InterpolatedDifferentialIKAction(DifferentialInverseKinematicsAction):
    """:class:`DifferentialInverseKinematicsAction` (one damped-least-squares step per physics step) for a position
    command, with the goal kept between policy steps and interpolated over each step's physics steps. The physics
    counterpart of :class:`~sim_base.newton_ik_action.InterpolatedNewtonIKAction` for PhysX, where Newton IK is not
    available.

    Goal: each policy step moves the tcp goal by ``scale * clip(action, -1, 1)`` [m] from where the previous step's
    goal was (root frame), kept within ``max_goal_lead`` [m] of the measured tcp: re-basing on the measured tcp (the
    relative mode) leaves the drives no error to hold against gravity. Interpolation: each physics step sets the next
    point of the line as the controller's absolute command (``use_relative_mode=False``), then the base solves.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        if "position" not in cfg.controller.command_type or cfg.controller.use_relative_mode:
            raise ValueError("InterpolatedDifferentialIKAction needs an absolute position command (use_relative_mode=False).")
        self._start = torch.zeros(self.num_envs, 3, device=self.device)
        self._end = torch.zeros_like(self._start)
        self._has_end = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._substep = 0

    @property
    def goal(self) -> torch.Tensor:
        """Tcp goal at the end of this policy step, in the env's root frame."""
        return self._end

    @property
    def has_goal(self) -> torch.Tensor:
        return self._has_end

    def process_actions(self, actions: torch.Tensor) -> None:
        self._raw_actions[:] = actions
        self._processed_actions[:] = torch.nan_to_num(actions, nan=0.0, posinf=1.0, neginf=-1.0).clamp(-1.0, 1.0)
        measured = self._compute_frame_pose()[0]
        self._start = torch.where(self._has_end.unsqueeze(-1), self._end, measured)
        lead = self.cfg.max_goal_lead
        self._end = torch.clamp(self._start + self._processed_actions * self.cfg.scale, measured - lead, measured + lead)
        self._has_end.fill_(True)
        self._substep = 0

    def apply_actions(self) -> None:
        self._substep += 1
        alpha = min(self._substep / self._env.cfg.decimation, 1.0)
        ee_pos, ee_quat = self._compute_frame_pose()
        self._ik_controller.set_command(self._start + (self._end - self._start) * alpha, ee_pos, ee_quat)
        super().apply_actions()

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        self._has_end[slice(None) if env_ids is None else env_ids] = False
