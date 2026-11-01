"""Drive helpers."""

import torch


class EffortLimitedDrive:
    """Cap the force of one joint's implicit PD drive at run time without touching the physics model.

    The position target is moved so that the drive output kp (target - q) - kd qd equals the clipped one.
    (Rewriting the sim effort limit instead, write_joint_effort_limit_to_sim_*, rewrites every joint's limit and
    locked the passive linkage of the Robotiq 2F-85 on PhysX.)
    """

    def __init__(self, robot, joint_name: str):
        self.robot = robot
        self.joint_id = robot.joint_names.index(joint_name)
        self.max_effort = float(robot.data.joint_effort_limits.torch[0, self.joint_id])
        self.kp = float(robot.data.joint_stiffness.torch[0, self.joint_id])
        self.kd = float(robot.data.joint_damping.torch[0, self.joint_id])

    def apply(self, target: torch.Tensor, limit: float) -> None:
        """Adjust ``target`` (num_envs x num_joints, env 0) in place for an effort limit of ``limit``."""
        if self.kp <= 0.0:
            return
        i = self.joint_id
        q = self.robot.data.joint_pos.torch[0, i]
        damping = self.kd * self.robot.data.joint_vel.torch[0, i]
        effort = torch.clamp(self.kp * (target[0, i] - q) - damping, -limit, limit)
        target[0, i] = q + (effort + damping) / self.kp
