"""Check the CRX-5iA + 2F-85 assembly: the gripper rides on the flange, opens and closes, fingers stay parallel.

    isaaclab -p /workspace/isaaclab_ws/src/fanuc_sim/scripts/check_crx5ia_2f85.py --engine physx
    isaaclab -p /workspace/isaaclab_ws/src/fanuc_sim/scripts/check_crx5ia_2f85.py --engine newton_mjwarp
"""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from sim_base.physics import ENGINES, physics_cfg

parser = argparse.ArgumentParser(description="Check CRX-5iA + 2F-85.")
parser.add_argument("--engine", choices=ENGINES, default="physx", help="Physics backend.")
add_launcher_args(parser)
args_cli = parser.parse_args()

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass, instantiate, replace

from fanuc_sim.robots import ROBOTIQ_NEWTON_IMPRATIO, ROBOTIQ_NEWTON_MAX_SOLVER_DT, crx5ia_2f85_cfg

DT = 0.002
CLOSED = 0.8  # finger_joint [rad], fully closed without an object
FAILED = []


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=3000.0))
    robot: ArticulationCfg = replace(crx5ia_2f85_cfg(args_cli.engine), prim_path="{ENV_REGEX_NS}/Robot")


def check(name: str, ok: bool, detail: str) -> None:
    print(f"  [{'OK' if ok else 'NG'}] {name}: {detail}")
    if not ok:
        FAILED.append(name)


def main():
    sim_cfg = sim_utils.SimulationCfg(
        device=args_cli.device,
        dt=DT,
        physics=physics_cfg(args_cli.engine, DT, ROBOTIQ_NEWTON_IMPRATIO, ROBOTIQ_NEWTON_MAX_SOLVER_DT),
    )
    with launch_simulation(sim_cfg, args_cli):
        # after Kit starts: OpenBLAS threads (numpy/scipy) crash Kit's startup fork
        global np, Rotation
        import numpy as np
        from scipy.spatial.transform import Rotation

        sim = SimulationContext(sim_cfg)
        scene = instantiate(SceneCfg(num_envs=1, env_spacing=2.0))
        robot = scene["robot"]
        sim.reset()
        print(f"== {args_cli.engine}\n  joints: {robot.joint_names}")
        finger = robot.joint_names.index("finger_joint")
        flange, base = robot.body_names.index("J6_link"), robot.body_names.index("base_link")
        tips = robot.body_names.index("left_fingertip"), robot.body_names.index("right_fingertip")

        robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch.clone())
        robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch.clone())
        robot.reset()
        target = robot.data.default_joint_pos.torch.clone()

        def run(finger_target: float, seconds: float):
            target[0, finger] = finger_target
            for _ in range(int(seconds / DT)):
                robot.actuators.target_command.set_position_index(value=target)
                robot.write_data_to_sim()
                sim.step(render=False)
                robot.update(DT)
            link = robot.data.body_link_pos_w.torch[0].cpu().numpy()
            quat = robot.data.body_link_quat_w.torch[0].cpu().numpy()  # x, y, z, w
            # everything in the flange frame; fingertips by CoM (the finger link frames all sit at the gripper base)
            R = Rotation.from_quat(quat[flange])
            tip_com = R.inv().apply(robot.data.body_com_pos_w.torch[0].cpu().numpy()[list(tips)] - link[flange])
            base_pos = R.inv().apply(link[base] - link[flange])
            base_rot = R.inv() * Rotation.from_quat(quat[base])
            return robot.data.joint_pos.torch[0].cpu().numpy(), tip_com, base_pos, base_rot

        for label, q in (("open", 0.0), ("closed", CLOSED), ("open again", 0.0)):
            jpos, (left, right), base_pos, base_rot = run(q, 1.5)
            arm_err = np.abs(jpos[:6] - robot.data.default_joint_pos.torch[0, :6].cpu().numpy()).max()
            gap = np.linalg.norm(left - right)
            print(f"  -- {label}: finger_joint {jpos[finger]:.3f} rad, fingertip (CoM) gap {gap * 1e3:.1f} mm")
            check(f"{label}: arm holds", arm_err < 0.01, f"max joint error {arm_err:.4f} rad")
            mount_err = np.linalg.norm(base_pos - [0.145, 0.0, 0.0]) * 1e3
            rot_err = np.degrees((base_rot * Rotation.from_euler("y", -90, degrees=True)).magnitude())
            check(f"{label}: gripper on the flange", mount_err < 1.0 and rot_err < 0.5,
                  f"base offset error {mount_err:.2f} mm, {rot_err:.2f} deg")
            check(f"{label}: finger_joint reaches target", abs(jpos[finger] - q) < 0.05, f"{jpos[finger]:.3f} rad")
            # parallel grip: both tips mirror each other about the gripper axis (flange x)
            check(f"{label}: fingers symmetric", abs(left[1] + right[1] - 2 * base_pos[1]) < 2e-3
                  and abs(left[0] - right[0]) < 2e-3,
                  f"tips (flange frame) L {np.round(left * 1e3, 1)} R {np.round(right * 1e3, 1)} mm")

        print("== ALL OK" if not FAILED else f"== NG: {FAILED}")


if __name__ == "__main__":
    main()
