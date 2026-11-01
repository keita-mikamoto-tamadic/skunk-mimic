"""SimApp: the sim loop shared by the robot scripts."""

import argparse
import time

import torch

from isaaclab.app import add_launcher_args, launch_simulation

from .bridge import Bridge
from .dora_bridge import DoraBridge
from .physics import ENGINES, physics_cfg
from .ros_bridge import RosBridge


class SimApp:
    """One robot in Isaac Sim, joint position targets in and joint states out through a :class:`Bridge`.

    Subclass and override the hooks; then ``MyApp().run()``. Common options: ``--engine --dt --render_hz
    --no_realtime --bridge {none,ros,dora}`` (+ the bridges' and Isaac Lab's launcher options).
    """

    description = "Isaac Lab robot sim."
    dt = 0.002  # default --dt [s]
    render_hz = 60.0  # default --render_hz
    bridge = "none"  # default --bridge
    camera = ((1.5, 1.5, 1.2), (0.0, 0.0, 0.5))  # eye, target
    bridges: dict[str, type[Bridge]] = {"none": Bridge, "ros": RosBridge, "dora": DoraBridge}

    def __init__(self, argv: list[str] | None = None):
        parser = argparse.ArgumentParser(description=self.description)
        # not "--physics": launch_simulation() would replace our physics cfg with its own default for that key
        parser.add_argument("--engine", choices=ENGINES, default="physx", help="Physics backend.")
        parser.add_argument("--dt", type=float, default=self.dt, help="Physics step [s].")
        parser.add_argument("--render_hz", type=float, default=self.render_hz, help="Rendering rate [Hz].")
        parser.add_argument("--no_realtime", action="store_true", help="Step as fast as possible instead of wall-clock pace.")
        parser.add_argument("--bridge", choices=list(self.bridges), default=self.bridge, help="Joint command / state transport.")
        for bridge in self.bridges.values():
            bridge.add_arguments(parser)
        self.add_arguments(parser)
        add_launcher_args(parser)
        self.args = parser.parse_args(argv)

    # ---- hooks -------------------------------------------------------------------------------------------------

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        """Robot-specific options."""

    def robot_cfg(self):
        """The robot's ArticulationCfg (prim_path is set by the app)."""
        raise NotImplementedError

    def scene_assets(self) -> dict:
        """Assets besides the robot, by scene name. Default: ground plane and dome light."""
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg

        return {
            "ground": AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg(color=(0.3, 0.3, 0.3))),
            "light": AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0)),
        }

    def newton_impratio(self) -> float:
        return 1.0

    def newton_max_solver_dt(self) -> float | None:
        """Largest Newton solver step the robot tolerates; a larger --dt is split into substeps."""
        return None

    def joint_name_map(self) -> dict[str, str]:
        """Isaac joint name -> name on the bridge side (default: the same)."""
        return {}

    def on_start(self) -> None:
        """After the scene is reset and the bridge created (self.sim, self.robot, self.bridge are set)."""

    def pre_step(self, target: torch.Tensor) -> None:
        """Adjust the position targets (1 x num_joints) before they are written to the sim."""

    def post_step(self) -> None:
        """After each physics step and publish."""

    # ---- run ---------------------------------------------------------------------------------------------------

    def run(self) -> None:
        import isaaclab.sim as sim_utils
        from isaaclab.scene import InteractiveSceneCfg
        from isaaclab.sim import SimulationContext
        from isaaclab.utils import instantiate, replace

        args = self.args
        sim_cfg = sim_utils.SimulationCfg(
            device=args.device,
            dt=args.dt,
            render_interval=max(1, round(1.0 / (args.render_hz * args.dt))),
            physics=physics_cfg(args.engine, args.dt, self.newton_impratio(), self.newton_max_solver_dt()),
        )
        with launch_simulation(sim_cfg, args):
            self.sim = SimulationContext(sim_cfg)
            self.sim.set_camera_view(*self.camera)
            # InteractiveScene is required for Newton (it builds its model from the clone plan)
            scene_cfg = InteractiveSceneCfg(num_envs=1, env_spacing=2.0)
            for name, cfg in self.scene_assets().items():
                setattr(scene_cfg, name, cfg)
            scene_cfg.robot = replace(self.robot_cfg(), prim_path="{ENV_REGEX_NS}/Robot")
            self.scene = instantiate(scene_cfg)
            self.robot = self.scene["robot"]
            self.sim.reset()
            # start in the configured initial pose: the articulation appears at its USD joint state (all 0) otherwise
            # and drives to the pose from there, through whatever is in the way (the RSARM forearm hits the table)
            default_pos = self.robot.data.default_joint_pos.torch
            self.robot.write_joint_position_to_sim_index(position=default_pos)
            self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(default_pos))
            self.robot.set_joint_position_target_index(target=default_pos)
            print("[INFO] Setup complete.")
            self._loop()

    def _loop(self) -> None:
        sim, robot, args = self.sim, self.robot, self.args
        dt = sim.get_physics_dt()
        self.bridge = self.bridges[args.bridge](args, robot.joint_names, self.joint_name_map())
        print(f"[INFO] joints: {self.bridge.names}")
        # hold the initial pose until the first command arrives
        self.bridge.target = robot.data.default_joint_pos.torch[0].tolist()
        self.on_start()
        self.bridge.start()

        render_interval = sim.cfg.render_interval  # SimulationContext.step() renders every call otherwise
        step = 0
        self.sim_time = 0.0
        wall_start = time.perf_counter()
        rtf_wall, rtf_sim = wall_start, 0.0
        try:
            while sim.is_running():
                target = torch.tensor([self.bridge.target], device=sim.device)
                self.pre_step(target)
                robot.actuators.target_command.set_position_index(value=target)
                robot.write_data_to_sim()
                step += 1
                sim.step(render=(step % render_interval == 0))
                robot.update(dt)
                self.sim_time += dt
                self.bridge.publish(
                    self.sim_time,
                    robot.data.joint_pos.torch[0].cpu(),
                    robot.data.joint_vel.torch[0].cpu(),
                    robot.actuators.applied_effort.torch[0].cpu(),
                )
                self.post_step()

                if not args.no_realtime:
                    lag = self.sim_time - (time.perf_counter() - wall_start)
                    if lag > 0:
                        time.sleep(lag)
                now = time.perf_counter()
                if now - rtf_wall >= 5.0:
                    print(f"[INFO] real-time factor: {(self.sim_time - rtf_sim) / (now - rtf_wall):.2f}")
                    rtf_wall, rtf_sim = now, self.sim_time
        finally:
            self.bridge.shutdown()
