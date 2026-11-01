"""Spawn the flywheel robot standing on the floor in Isaac Sim and keep it running (sim_base.SimApp).

The wheel is torque driven (no position gains), so the bridges' joint position targets do nothing and the robot
falls over to its side: this is to check the model and its contacts. Balancing is the RL task Flywheel-Balance.

    isaaclab -p /workspace/isaaclab_ws/src/flywheel_sim/scripts/view_flywheel.py --viz kit
    isaaclab -p /workspace/isaaclab_ws/src/flywheel_sim/scripts/view_flywheel.py --engine newton_mjwarp --viz newton
"""

from sim_base import SimApp

from flywheel_sim.robots import FLYWHEEL_CFG


class FlywheelSimApp(SimApp):
    description = "View the flywheel robot."
    camera = ((0.35, 0.35, 0.2), (0.0, 0.0, 0.06))

    def robot_cfg(self):
        return FLYWHEEL_CFG


if __name__ == "__main__":
    FlywheelSimApp().run()
