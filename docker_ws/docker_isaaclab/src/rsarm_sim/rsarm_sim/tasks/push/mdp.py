"""MDP of pushing the blue box over the table to a target point with RSARM's tcp sphere (the scene and the env are in
push_env_cfg.py).

Success: the target point on the table lies inside the box's bottom face (whichever face is down, any yaw) while
the box rests there. Failure, ending the episode with a penalty: a robot link touching the table, two
non-adjacent robot links touching (self collision), or the box falling off the table. Shaping follows the lift and
place tasks: differences of potentials only (box to target, tcp to the push point), so hovering earns nothing.
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
from isaaclab.utils.math import matrix_from_quat, quat_apply, quat_apply_inverse, subtract_frame_transforms

from rsarm_sim.robots import BLUE_BOX_SIZE, TABLE_TOP_Z
from isaaclab_newton.controllers.ik.newton_ik_objectives_cfg import NewtonIKPoseObjectiveCfg
from isaaclab_newton.controllers.ik.newton_ik_solver_cfg import NewtonIKSolverCfg

from isaaclab.controllers import DifferentialIKControllerCfg

from sim_base.actions import (
    InterpolatedDifferentialIKActionCfg,
    InterpolatedJointPositionActionCfg,
    InterpolatedNewtonIKActionCfg,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import RewardTermCfg, TerminationTermCfg

ROBOT = SceneEntityCfg("robot")
BOX = SceneEntityCfg("blue_box")
# target point on the table plate (robot frame) [m]
TARGET = (0.25, -0.1, TABLE_TOP_Z)
BOX_HALF = BLUE_BOX_SIZE / 2
# tcp: the forearm's tip sphere (A400 MeshInstance_3, 29 mm), in the A400 link frame [m]
TCP_LINK = "A400"
TCP_OFFSET = (0.0, 0.025, -0.200)
TCP_RADIUS = 0.0145
# push point: behind the box, opposite the target, this far beyond touching [m]
PUSH_MARGIN = 0.005
# success: target inside the bottom face, the box resting (bottom within this of the table) and still, N steps
REST_TOLERANCE = 0.01
STILL_MAX_SPEED = 0.05
SUCCESS_STEPS = 5
# the work area: the front table plate (MeshInstance_459, 3 mm), robot frame [m]. The rear plate (_460, y 0.148 to
# 0.548) is behind the robot's mount and not used: the box past the front plate's edges counts as off the table.
PLATE_X = (0.0, 0.35)
PLATE_Y = (-0.252, 0.148)
# the box fell off the table: its center this far below the plate, or beyond its edge (it tips over there) [m]
DROP_DEPTH = 0.05
# box spawn: on the plate with this margin past the box's half diagonal, within the arm's push reach of the shoulder
# (horizontal), and away from the target so that an episode never starts solved [m]
SPAWN_EDGE_MARGIN = BOX_HALF * 2**0.5 + 0.01
SHOULDER_XY = (0.0, -0.045)
SPAWN_REACH = (0.08, 0.25)
SPAWN_MIN_TARGET_DISTANCE = 0.08
CONTACT_THRESHOLD = 1.0  # [N]

# contact sensors (scene name -> (link, filter prims under the env)). One sensor per link and kind: PhysX filters
# contacts per single body, and a filter body with several shapes takes several columns, so kinds are not mixed in
# one sensor. A000 has no collider; adjacent links touch at the joints, so only non-adjacent pairs count as self
# collision.
TABLE_COLLIDERS = ["Table/Cube", "Table/MeshInstance_405/Body1_collision"]
# the PhysX variant's table: two static cuboids instead of the table USD (466 meshes per env took 60 GB of host
# memory at 4096 envs): the front plate (3 cm thick, its top on the plate's) and the robot's mount post, as
# (scene name, center, size) in the robot frame [m]
SIMPLE_TABLE = [
    ("table_plate", (0.175, -0.052, TABLE_TOP_Z - 0.015), (0.35, 0.40, 0.03)),
    ("table_post", (0.0, 0.083, -0.133), (0.06, 0.03, 0.32)),
]
SIMPLE_TABLE_COLLIDERS = [f"{name}/geometry/mesh" for name, _, _ in SIMPLE_TABLE]
CONTACT_SENSORS = {
    **{f"{link}_table": (link, TABLE_COLLIDERS) for link in ("A100", "A200", "A300", "A400")},
    "A300_self": ("A300", ["Robot/A100"]),
    "A400_self": ("A400", ["Robot/A100", "Robot/A200"]),
    "A400_box": ("A400", ["BlueBox"]),
}


def _touching(env: ManagerBasedRLEnv, sensors) -> torch.Tensor:
    """Any of the named sensors reports a filtered contact above CONTACT_THRESHOLD."""
    hit = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    for name in sensors:
        force = torch.norm(env.scene.sensors[name].data.normal_force_matrix_w.torch, dim=-1)  # (envs, 1, filters)
        hit |= (force > CONTACT_THRESHOLD).flatten(1).any(dim=-1)
    return hit


# ---- geometry ------------------------------------------------------------------------------------------------------


def tcp_pos_w(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Center of the forearm's tip sphere."""
    robot = env.scene[ROBOT.name]
    link = robot.find_bodies([TCP_LINK])[0][0]
    offset = torch.tensor(TCP_OFFSET, device=env.device).expand(env.num_envs, 3)
    return robot.data.body_pos_w.torch[:, link] + quat_apply(robot.data.body_quat_w.torch[:, link], offset)


def target_w(env: ManagerBasedRLEnv) -> torch.Tensor:
    return env.scene.env_origins + torch.tensor(TARGET, device=env.device)


def _box_to_target_xy(env: ManagerBasedRLEnv) -> torch.Tensor:
    return target_w(env)[:, :2] - env.scene[BOX.name].data.root_pos_w.torch[:, :2]


def push_point_w(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Where the tcp sphere touches the box to push it toward the target: behind it, at its center height."""
    box = env.scene[BOX.name].data.root_pos_w.torch
    direction = torch.nn.functional.normalize(_box_to_target_xy(env), dim=-1)
    point = box.clone()
    point[:, :2] -= direction * (BOX_HALF + TCP_RADIUS + PUSH_MARGIN)
    return point


def target_in_bottom_face(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The target point lies on the box's downward face (any face, any yaw), and the box rests on the table."""
    box = env.scene[BOX.name]
    quat = box.data.root_quat_w.torch
    local = quat_apply_inverse(quat, target_w(env) - box.data.root_pos_w.torch)
    # the down face's axis: the box axis most aligned with world z
    down = matrix_from_quat(quat)[:, 2, :].abs().argmax(dim=-1)
    on_axis = local.gather(1, down.unsqueeze(1)).squeeze(1).abs()
    in_face = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
    for k in range(3):
        in_face &= (down == k) | (local[:, k].abs() <= BOX_HALF)
    return in_face & ((on_axis - BOX_HALF).abs() < REST_TOLERANCE)


# ---- progress / success --------------------------------------------------------------------------------------------


class PushSuccessTerm(ManagerTermBase):
    """Success once the target has been inside the bottom face, with the box still, for SUCCESS_STEPS steps."""

    def __init__(self, cfg: TerminationTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._count = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        env._push_success = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        ids = slice(None) if env_ids is None else env_ids
        self._count[ids] = 0
        self._env._push_success[ids] = False

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        still = torch.norm(env.scene[BOX.name].data.root_lin_vel_w.torch, dim=1) < STILL_MAX_SPEED
        placed = target_in_bottom_face(env) & still
        self._count = torch.where(placed, self._count + 1, torch.zeros_like(self._count))
        env._push_success = self._count >= SUCCESS_STEPS
        log = env.extras.setdefault("log", {})
        log["Metrics/in_face_rate"] = placed.float().mean()
        log["Metrics/box_to_target_m"] = torch.norm(_box_to_target_xy(env), dim=-1).mean()
        return env._push_success.clone()


def success_bonus(env: ManagerBasedRLEnv) -> torch.Tensor:
    return env._push_success.float()


# ---- failures (terminations, penalized by the same functions as rewards) ------------------------------------------


def table_contact(env: ManagerBasedRLEnv) -> torch.Tensor:
    """A robot link pushes on the table."""
    return _touching(env, [n for n in CONTACT_SENSORS if n.endswith("_table")])


def self_collision(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Two non-adjacent robot links touch."""
    return _touching(env, [n for n in CONTACT_SENSORS if n.endswith("_self")])


def box_dropped(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The box fell off the table (below the plate, or its center beyond the plate's edge), or is not finite."""
    local = env.scene[BOX.name].data.root_pos_w.torch - env.scene.env_origins
    fell = local[:, 2] < TABLE_TOP_Z - DROP_DEPTH
    off = (local[:, 0] < PLATE_X[0]) | (local[:, 0] > PLATE_X[1]) | (local[:, 1] < PLATE_Y[0]) | (local[:, 1] > PLATE_Y[1])
    return fell | off | ~torch.isfinite(local).all(dim=-1)


def unstable_robot(env: ManagerBasedRLEnv, max_joint_vel: float = 12.0) -> torch.Tensor:
    vel = env.scene[ROBOT.name].data.joint_vel.torch
    return (~torch.isfinite(vel).all(dim=-1)) | (vel.abs().amax(dim=1) > max_joint_vel)


# ---- shaping -------------------------------------------------------------------------------------------------------


class DistanceProgressReward(ManagerTermBase):
    """Pay the decrease of a distance, in units of ``scale`` [m] and clipped to +-1 (a difference of potentials:
    moving back costs what moving closer paid, hovering pays nothing). ``distance`` picks which one."""

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._previous = torch.zeros(self.num_envs, device=self.device)
        self._has_previous = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._has_previous[slice(None) if env_ids is None else env_ids] = False

    def __call__(self, env: ManagerBasedRLEnv, distance: str, scale: float = 0.01) -> torch.Tensor:
        d = DISTANCES[distance](env)
        progress = ((self._previous - d) / scale).clamp(-1.0, 1.0)
        progress = torch.where(self._has_previous, progress, torch.zeros_like(progress))
        self._previous.copy_(d)
        self._has_previous.fill_(True)
        return progress


DISTANCES = {
    # box center to the target, horizontal: the task itself
    "box_to_target": lambda env: torch.norm(_box_to_target_xy(env), dim=-1),
    # tcp to the push point: gets the sphere behind the box
    "tcp_to_push_point": lambda env: torch.norm(tcp_pos_w(env) - push_point_w(env), dim=-1),
}


# ---- events --------------------------------------------------------------------------------------------------------


def reset_box_on_table(env: ManagerBasedRLEnv, env_ids: torch.Tensor | slice | None, tries: int = 32) -> None:
    """Place the box at random on the plate (SPAWN_* constraints, rejection sampled), any yaw, at rest.

    An env whose every try fails (practically never) keeps the default position.
    """
    ids = torch.arange(env.num_envs, device=env.device)
    if isinstance(env_ids, slice):  # the first reset passes a slice
        ids = ids[env_ids]
    elif env_ids is not None:
        ids = torch.as_tensor(env_ids, device=env.device)
    n = len(ids)
    lo = torch.tensor([PLATE_X[0] + SPAWN_EDGE_MARGIN, PLATE_Y[0] + SPAWN_EDGE_MARGIN], device=env.device)
    hi = torch.tensor([PLATE_X[1] - SPAWN_EDGE_MARGIN, PLATE_Y[1] - SPAWN_EDGE_MARGIN], device=env.device)
    box = env.scene[BOX.name]
    xy = box.data.default_root_pose.torch[ids, :2].clone()
    pending = torch.ones(n, dtype=torch.bool, device=env.device)
    shoulder = torch.tensor(SHOULDER_XY, device=env.device)
    target = torch.tensor(TARGET[:2], device=env.device)
    for _ in range(tries):
        sample = lo + (hi - lo) * torch.rand(n, 2, device=env.device)
        reach = torch.norm(sample - shoulder, dim=-1)
        ok = (reach > SPAWN_REACH[0]) & (reach < SPAWN_REACH[1]) & (torch.norm(sample - target, dim=-1) > SPAWN_MIN_TARGET_DISTANCE)
        take = pending & ok
        xy[take] = sample[take]
        pending &= ~ok
        if not pending.any():
            break
    yaw = (torch.rand(n, device=env.device) * 2 - 1) * torch.pi
    pose = torch.zeros(n, 7, device=env.device)
    pose[:, :2] = xy
    pose[:, 2] = TABLE_TOP_Z + BOX_HALF
    pose[:, :3] += env.scene.env_origins[ids]
    pose[:, 5], pose[:, 6] = torch.sin(yaw / 2), torch.cos(yaw / 2)  # (x, y, z, w) about z
    box.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
    box.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=env.device), env_ids=ids)


# ---- observations --------------------------------------------------------------------------------------------------


def _root_frame(env: ManagerBasedRLEnv, pos_w: torch.Tensor) -> torch.Tensor:
    robot = env.scene[ROBOT.name]
    return subtract_frame_transforms(robot.data.root_pos_w.torch, robot.data.root_quat_w.torch, pos_w)[0]


def tcp_pos_b(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _root_frame(env, tcp_pos_w(env))


def box_pos_b(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _root_frame(env, env.scene[BOX.name].data.root_pos_w.torch)


def box_quat_b(env: ManagerBasedRLEnv) -> torch.Tensor:
    robot, box = env.scene[ROBOT.name], env.scene[BOX.name]
    return subtract_frame_transforms(
        robot.data.root_pos_w.torch, robot.data.root_quat_w.torch, box.data.root_pos_w.torch, box.data.root_quat_w.torch
    )[1]


def box_to_target(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Target minus box center (the robot base is not rotated, so world axes are its axes)."""
    return target_w(env) - env.scene[BOX.name].data.root_pos_w.torch


def arm_target_rel(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Joint position targets the drives hold, relative to the default pose: hidden state otherwise (either action)."""
    robot = env.scene[ROBOT.name]
    return robot.actuators.target_command.position.torch - robot.data.default_joint_pos.torch


def tcp_goal_offset(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Tcp goal of the IK action minus the tcp, root frame [m] (0 until the first step after a reset sets a goal)."""
    term = env.action_manager.get_term("arm_action")
    tcp = tcp_pos_b(env)
    return torch.where(term.has_goal.unsqueeze(-1), term.goal - tcp, torch.zeros_like(tcp))


def tcp_box_contact_force(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Contact force between the forearm (A400, the tcp sphere's link) and the box, world frame [N] (critic only)."""
    return env.scene.sensors["A400_box"].data.normal_force_matrix_w.torch.sum(dim=(1, 2))


def box_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    return env.scene[BOX.name].data.root_lin_vel_w.torch


# ---- configs -------------------------------------------------------------------------------------------------------


@configclass
class ActionsCfg:
    """The policy moves the tcp by up to 1 cm per 50 Hz step (0.5 m/s), relative to where it is; every 5 ms physics
    step takes the next point of the line and Isaac Lab's Newton IK (Levenberg-Marquardt) solves to it. Position
    only: the tip is a sphere, and four joints cannot hold an orientation as well."""

    arm_action = InterpolatedNewtonIKActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        objectives=[
            NewtonIKPoseObjectiveCfg(
                body_name=TCP_LINK,
                body_offset_pos=TCP_OFFSET,
                command_type="position",
                use_relative_mode=True,
                scale=0.01,
                position_weight=1.0,
                rotation_weight=0.0,
            ),
        ],
        controller=NewtonIKSolverCfg(optimizer="lm", jacobian_mode="analytic", iterations=24),
    )


@configclass
class PhysxActionsCfg:
    """As :class:`ActionsCfg` for PhysX, where Newton IK is not available: Isaac Lab's differential IK (one damped
    least squares step per physics step) to the interpolated tcp goal."""

    arm_action = InterpolatedDifferentialIKActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        body_name=TCP_LINK,
        body_offset=InterpolatedDifferentialIKActionCfg.OffsetCfg(pos=TCP_OFFSET),
        scale=0.01,
        controller=DifferentialIKControllerCfg(
            command_type="position", use_relative_mode=False, ik_method="dls", ik_params={"lambda_val": 0.05}
        ),
    )


@configclass
class JointActionsCfg:
    """Joint targets moved by up to 0.02 rad per 50 Hz step (1 rad/s), interpolated over the 5 ms physics steps."""

    arm_action = InterpolatedJointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.02)


@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        arm_target = ObsTerm(func=arm_target_rel)
        # positions in the robot root frame [m]; the box pose is always observable
        tcp_pos = ObsTerm(func=tcp_pos_b)
        box_pos = ObsTerm(func=box_pos_b)
        box_quat = ObsTerm(func=box_quat_b)
        box_to_target = ObsTerm(func=box_to_target)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Privileged terms for the critic only (on top of the policy group)."""

        tcp_box_force = ObsTerm(func=tcp_box_contact_force)
        box_vel = ObsTerm(func=box_lin_vel)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class IkPolicyCfg(PolicyCfg):
        # where the IK action is taking the tcp: hidden state otherwise
        tcp_goal = ObsTerm(func=tcp_goal_offset)

    policy: PolicyCfg = IkPolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    # scene to its default state first (overriding the events drops Isaac Lab's default reset), then the box: anywhere
    # on the plate it cannot fall from and the arm can push it, any yaw
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")
    reset_box = EventTerm(func=reset_box_on_table, mode="reset")


@configclass
class RewardCfg:
    # dense: the box closed toward the target (1 cm = 1), the tcp toward the push point
    box_progress = RewTerm(func=DistanceProgressReward, weight=1.0, params={"distance": "box_to_target"})
    reach = RewTerm(func=DistanceProgressReward, weight=0.5, params={"distance": "tcp_to_push_point"})
    # success once (ends the episode)
    success = RewTerm(func=success_bonus, weight=200.0)
    # failures (each ends the episode)
    table_contact = RewTerm(func=table_contact, weight=-20.0)
    self_collision = RewTerm(func=self_collision, weight=-20.0)
    box_dropped = RewTerm(func=box_dropped, weight=-20.0)
    # regularization
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.002)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.0002)


@configclass
class TerminationsCfg:
    # first: the success bonus reads what it latches
    success = DoneTerm(func=PushSuccessTerm)
    table_contact = DoneTerm(func=table_contact)
    self_collision = DoneTerm(func=self_collision)
    box_dropped = DoneTerm(func=box_dropped)
    unstable_robot = DoneTerm(func=unstable_robot)
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
