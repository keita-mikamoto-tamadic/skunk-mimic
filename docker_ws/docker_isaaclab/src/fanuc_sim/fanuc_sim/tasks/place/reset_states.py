"""Recorded mid-task start states for the place task, as the SO-101 vial tutorial's reset dataset
(IsaacLabTutorial, place_vial/reset): training episodes start from any phase, so carrying and placing are practised
from the first iteration instead of only after a grasp is first discovered.

The states come from the trained lift policy run on the lift task (physics-validated: every row is a state the
simulation actually reached). Record them once, through Isaac Lab's play CLI:

    isaaclab play --rl_library rsl_rl --task Fanuc-Lift-Cube --num_envs 50 \
      --external_callback fanuc_sim.tasks.place.reset_states.record \
      --checkpoint /workspace/isaaclab/logs/rsl_rl/crx_lift/<run>/model_<n>.pt

It writes reset_states.pt next to this file and exits.
"""

from __future__ import annotations

import os
from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ManagerTermBase

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import EventTermCfg

RESET_STATES = Path(__file__).with_name("reset_states.pt")
PHASES = ("home", "pregrasp", "grasped", "lifted")
"""0: the default reset (not recorded), 1: tcp at the box, not grasped yet, 2: grasped on the ground, 3: lifted."""
# pregrasp: the lift policy closes the fingers to about the box width on the way in (finger_joint 0.27 rad at
# 7-10 cm, 0.35 at 0-3 cm) and squeezes last, so "not yet squeezing" is below the 0.29 rad of the squeeze plus margin
PREGRASP_MAX_FINGER = 0.36
ROWS_PER_PHASE = 512
# chance to record a matching state on a step: spreads the rows over episodes rather than consecutive steps
RECORD_PROBABILITY = 0.05


# ---- recording (lift task, lift policy) ----------------------------------------------------------------------------


class _Recorder:
    def __init__(self):
        self.rows = {phase: [] for phase in range(1, len(PHASES))}

    def step(self, env: ManagerBasedRLEnv) -> None:
        from fanuc_sim.tasks.lift import mdp as lift

        robot, red = env.scene["robot"], env.scene["red_box"]
        progress = lift._progress(env)
        red_pos = red.data.root_pos_w.torch
        height = lift._height(env, lift.RED_BOX) - lift.RED_REST_HEIGHT
        finger_id = robot.joint_names.index("finger_joint")
        finger = robot.data.joint_pos.torch[:, finger_id]
        near = torch.norm(lift.tcp_pos_w(env) - red_pos, dim=1) < 0.05
        phase_masks = {
            1: ~progress.grasped & near & (finger < PREGRASP_MAX_FINGER) & (height < 0.005),
            2: progress.grasped & progress.held & (height < 0.01),
            3: progress.held & (height > 0.04),
        }
        pose = torch.cat((red_pos - env.scene.env_origins, red.data.root_quat_w.torch), dim=-1)
        arm_target = env.action_manager.get_term("arm_action").target
        finger_target = robot.data.joint_pos_target.torch[:, finger_id]
        for phase, mask in phase_masks.items():
            rows = self.rows[phase]
            pick = mask & (torch.rand(env.num_envs, device=env.device) < RECORD_PROBABILITY)
            for i in pick.nonzero().flatten().tolist():
                if len(rows) < ROWS_PER_PHASE:
                    rows.append((robot.data.joint_pos.torch[i].cpu(), arm_target[i].cpu(), finger_target[i].cpu(),
                                 pose[i].cpu()))
        counts = {PHASES[p]: len(r) for p, r in self.rows.items()}
        if env.common_step_counter % 100 == 0:
            print(f"[reset_states] recorded {counts}")
        if all(len(r) >= ROWS_PER_PHASE for r in self.rows.values()):
            self.save()
            print(f"[reset_states] wrote {RESET_STATES} {counts}")
            os._exit(0)  # the play loop has no stop condition

    def save(self) -> None:
        phases = [p for p, rows in self.rows.items() for _ in rows]
        flat = [row for rows in self.rows.values() for row in rows]
        torch.save(
            {
                "phases": PHASES,
                "phase": torch.tensor(phases, dtype=torch.long),
                "joint_pos": torch.stack([r[0] for r in flat]),
                "arm_target": torch.stack([r[1] for r in flat]),
                "finger_target": torch.stack([r[2] for r in flat]),
                "red_box_pose": torch.stack([r[3] for r in flat]),  # env-local position, quaternion (x, y, z, w)
            },
            RESET_STATES,
        )


def record() -> None:
    """--external_callback for play on Fanuc-Lift-Cube: register the tasks and record start states each step."""
    import fanuc_sim.tasks  # noqa: F401
    from fanuc_sim.tasks.lift.mdp import LiftSuccessTerm

    recorder = _Recorder()
    call = LiftSuccessTerm.__call__

    def recording_call(self, env):
        done = call(self, env)  # advances the lift milestones first
        recorder.step(env)
        return done

    LiftSuccessTerm.__call__ = recording_call


# ---- reset (place task) --------------------------------------------------------------------------------------------


class ResetFromStates(ManagerTermBase):
    """Start each resetting env from a phase drawn with ``phase_weights`` (one per PHASES entry), then a random
    recorded state of that phase. "home" leaves the state of the events before it (the default reset).

    The recorded arm target is handed to the arm action, and the grasp milestone to the place progress, through env
    attributes their resets read (the action and termination managers reset after the events).
    """

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        if not RESET_STATES.exists():
            raise FileNotFoundError(f"{RESET_STATES} is missing: record it first (see {__name__}'s docstring)")
        states = torch.load(RESET_STATES)
        self.phase = states["phase"].to(env.device)
        self.states = {k: v.to(env.device) for k, v in states.items() if isinstance(v, torch.Tensor) and k != "phase"}
        robot = env.scene["robot"]
        self._finger_id = robot.joint_names.index("finger_joint")
        env._reset_arm_target = torch.zeros((env.num_envs, self.states["arm_target"].shape[1]), device=env.device)
        env._use_reset_arm_target = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        env._reset_grasped = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        env._reset_phase = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)  # for logs and checks

    def __call__(self, env: ManagerBasedRLEnv, env_ids: torch.Tensor | slice | None, phase_weights: Sequence[float]):
        all_ids = torch.arange(env.num_envs, device=env.device)
        if env_ids is None or isinstance(env_ids, slice):  # the first reset passes a slice
            ids = all_ids[env_ids if isinstance(env_ids, slice) else slice(None)]
        else:
            ids = torch.as_tensor(env_ids, device=env.device)
        weights = torch.as_tensor(phase_weights, dtype=torch.float, device=env.device)
        phase = torch.multinomial(weights, len(ids), replacement=True)
        env._use_reset_arm_target[ids] = False
        env._reset_grasped[ids] = False
        env._reset_phase[ids] = phase
        loaded = phase > 0
        if not loaded.any():
            return
        ids, phase = ids[loaded], phase[loaded]
        # a random recorded row of each env's phase
        match = (self.phase.unsqueeze(0) == phase.unsqueeze(1)).float()
        rows = torch.multinomial(match, 1).squeeze(1)

        robot, red = env.scene["robot"], env.scene["red_box"]
        joint_pos = self.states["joint_pos"][rows]
        joint_target = joint_pos.clone()
        joint_target[:, self._finger_id] = self.states["finger_target"][rows]
        robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=ids)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(joint_pos), env_ids=ids)
        robot.set_joint_position_target_index(target=joint_target, env_ids=ids)
        pose = self.states["red_box_pose"][rows].clone()
        pose[:, :3] += env.scene.env_origins[ids]
        red.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
        red.write_root_velocity_to_sim_index(root_velocity=torch.zeros((len(ids), 6), device=env.device), env_ids=ids)

        env._reset_arm_target[ids] = self.states["arm_target"][rows]
        env._use_reset_arm_target[ids] = True
        env._reset_grasped[ids] = phase >= PHASES.index("grasped")
