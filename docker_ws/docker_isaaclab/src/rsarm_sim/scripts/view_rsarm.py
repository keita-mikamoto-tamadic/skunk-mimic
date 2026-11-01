"""Spawn the table and RSARM in Isaac Sim and keep it running (sim_base.SimApp).

The arm holds its initial pose; with --bridge ros it takes joint position targets over ROS 2 like the FANUC bridge.

    isaaclab -p /workspace/isaaclab_ws/src/rsarm_sim/scripts/view_rsarm.py --viz kit
    # right-click drag in the Newton viewer works only with Newton physics
    isaaclab -p /workspace/isaaclab_ws/src/rsarm_sim/scripts/view_rsarm.py --engine newton_mjwarp --viz newton
    isaaclab -p /workspace/isaaclab_ws/src/rsarm_sim/scripts/view_rsarm.py --bridge ros --viz kit
"""

from sim_base import SimApp

from rsarm_sim.robots import BLUE_BOX_CFG, RSARM_CFG, TABLE_CFG


class RsarmSimApp(SimApp):
    description = "View RSARM."
    camera = ((1.0, -1.2, 0.4), (0.0, -0.1, -0.2))

    def robot_cfg(self):
        return RSARM_CFG

    def scene_assets(self):
        from isaaclab.assets import AssetBaseCfg
        from isaaclab.utils import replace

        assets = super().scene_assets()
        # floor under the table
        assets["ground"] = replace(assets["ground"], init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, -1.188)))
        assets["table"] = replace(TABLE_CFG, prim_path="{ENV_REGEX_NS}/Table")
        assets["blue_box"] = BLUE_BOX_CFG
        return assets

    def newton_max_solver_dt(self):
        return 0.002


if __name__ == "__main__":
    RsarmSimApp().run()
