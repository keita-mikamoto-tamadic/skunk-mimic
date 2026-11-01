"""Run the FANUC CRX-5iA with a Robotiq 2F-85 in Isaac Sim and bridge it to ROS 2 (sim_base.SimApp, --bridge ros
by default).

    pub  /clock, /isaac_joint_states (position / velocity)   sub  /isaac_joint_commands (position targets)

The Isaac finger_joint is robotiq_85_left_knuckle_joint on the ROS side and the effort of its command is the grip
force [N], as with the real Robotiq driver (ros2_robotiq_gripper).

.. code-block:: bash

    isaaclab -p /workspace/isaaclab_ws/src/fanuc_sim/scripts/sim_bridge.py
    isaaclab -p /workspace/isaaclab_ws/src/fanuc_sim/scripts/sim_bridge.py --engine newton_mjwarp --dt 0.01 --viz newton
    # extra scene assets by name (fanuc_sim.objects.ASSET_SETS), e.g. the red and yellow boxes
    isaaclab -p /workspace/isaaclab_ws/src/fanuc_sim/scripts/sim_bridge.py --assets boxes
    # without ROS: the robot holds its initial pose
    isaaclab -p /workspace/isaaclab_ws/src/fanuc_sim/scripts/sim_bridge.py --bridge none --viz kit
"""

from sim_base import EffortLimitedDrive, SimApp

from fanuc_sim.objects import ASSET_SETS
from fanuc_sim.robots import (
    ROBOTIQ_DRIVER_JOINT_NAMES,
    ROBOTIQ_NEWTON_IMPRATIO,
    ROBOTIQ_NEWTON_MAX_SOLVER_DT,
    crx5ia_2f85_cfg,
    robotiq_force_fraction,
)


class CrxSimApp(SimApp):
    description = "FANUC CRX-5iA + Robotiq 2F-85 in Isaac Sim <-> ROS 2."
    dt = 1.0 / 500.0  # fanuc_driver runs at 500 Hz
    bridge = "ros"

    def add_arguments(self, parser):
        parser.add_argument(
            "--assets", choices=["none", *ASSET_SETS], default="none", help="Scene assets besides the robot."
        )

    def robot_cfg(self):
        return crx5ia_2f85_cfg(self.args.engine)

    def scene_assets(self):
        return {**super().scene_assets(), **ASSET_SETS.get(self.args.assets, {})}

    def newton_impratio(self):
        return ROBOTIQ_NEWTON_IMPRATIO

    def newton_max_solver_dt(self):
        return ROBOTIQ_NEWTON_MAX_SOLVER_DT

    def joint_name_map(self):
        return ROBOTIQ_DRIVER_JOINT_NAMES

    def on_start(self):
        self.gripper = EffortLimitedDrive(self.robot, "finger_joint")
        print(f"[INFO] gripper drive: effort limit {self.gripper.max_effort:.2f}, kp {self.gripper.kp:.2f}, kd {self.gripper.kd:.2f}")

    def pre_step(self, target):
        force = self.bridge.effort.get(self.gripper.joint_id)
        if force is not None:
            self.gripper.apply(target, self.gripper.max_effort * robotiq_force_fraction(force))


if __name__ == "__main__":
    CrxSimApp().run()
