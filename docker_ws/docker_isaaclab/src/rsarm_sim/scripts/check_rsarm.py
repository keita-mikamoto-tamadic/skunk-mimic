"""Check the generated RSARM USD against the CAD CSVs, then in simulation.

1. static (pxr): body origins, mass / center of mass / inertia tensor, joint axes and anchors vs the CSVs
2. sim: hold the initial pose (RSARM_CFG.init_state), then move each joint alone by +0.5 rad and check
   that its child rotates about the expected world axis around its own origin

    isaaclab -p /workspace/isaaclab_ws/src/rsarm_sim/scripts/check_rsarm.py [--engine newton_mjwarp]
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation
from sim_base.physics import ENGINES, physics_cfg

parser = argparse.ArgumentParser(description="RSARM USD / simulation check.")
parser.add_argument("--engine", choices=ENGINES, default="physx")
parser.add_argument("--dir", default="/workspace/robot_file/rsarm_v0", type=Path)
parser.add_argument("--no_table", action="store_true", help="Do not spawn the table.")
add_launcher_args(parser)
args_cli = parser.parse_args()

import numpy as np
import torch
from scipy.spatial.transform import Rotation

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass, instantiate, replace

from rsarm_sim.robots import RSARM_CFG, TABLE_CFG

# pxr (and build_rsarm_usd, which imports it) is loaded only after Kit has started: importing the
# standalone pxr first breaks Kit's own USD bindings
sys.path.insert(0, str(Path(__file__).parent))

AXES = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0]), "Z": np.array([0, 0, 1.0])}
fails = []


def check(name: str, ok: bool, detail: str) -> None:
    print(f"  [{'OK' if ok else 'NG'}] {name}: {detail}")
    if not ok:
        fails.append(name)


def static_check() -> dict:
    from build_rsarm_usd import JOINTS, PARTS, read_csv
    from pxr import Usd, UsdGeom, UsdPhysics

    print("== static (USD vs CSV)")
    stage = Usd.Stage.Open(str(args_cli.dir / "rsarm.usd"))
    xc = UsdGeom.XformCache()
    csv = {name: read_csv(args_cli.dir / f) for name, f, _ in PARTS}
    for name, _, _ in PARTS:
        prim = stage.GetPrimAtPath(f"/rsarm/{name}")
        t = np.array(xc.GetLocalToWorldTransform(prim).ExtractTranslation())
        c = csv[name]
        check(f"{name} origin", np.allclose(t, c["origin"], atol=1e-6), f"{np.round(t * 1e3, 2)} mm")
        m = UsdPhysics.MassAPI(prim)
        com = np.array(m.GetCenterOfMassAttr().Get()) + t
        q = m.GetPrincipalAxesAttr().Get()
        R = Rotation.from_quat([*q.GetImaginary(), q.GetReal()]).as_matrix()
        inertia = R @ np.diag(m.GetDiagonalInertiaAttr().Get()) @ R.T
        check(f"{name} mass", abs(m.GetMassAttr().Get() - c["mass"]) < 1e-6, f"{m.GetMassAttr().Get():.4f} kg")
        check(f"{name} com", np.allclose(com, c["com_world"], atol=1e-6), f"{np.round(com * 1e3, 2)} mm (world)")
        err = np.abs(inertia - c["inertia"]).max() / np.abs(c["inertia"]).max()
        check(f"{name} inertia", err < 1e-4, f"max rel err {err:.1e}")
    for jname, parent, child, axis in JOINTS:
        j = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"/rsarm/joints/{jname}"))
        anchor = csv[parent]["origin"] + np.array(j.GetLocalPos0Attr().Get())
        ok = j.GetAxisAttr().Get() == axis and np.allclose(anchor, csv[child]["origin"], atol=1e-6)
        check(f"{jname} ({parent}->{child})", ok, f"axis {j.GetAxisAttr().Get()}, anchor {np.round(anchor * 1e3, 2)} mm")
    return csv


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(), init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, -1.3)))
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=3000.0))
    table: AssetBaseCfg | None = None if args_cli.no_table else replace(TABLE_CFG, prim_path="{ENV_REGEX_NS}/Table")
    robot: ArticulationCfg = replace(RSARM_CFG, prim_path="{ENV_REGEX_NS}/Robot")


def main():
    dt = 0.005
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=dt, physics=physics_cfg(args_cli.engine, dt))
    with launch_simulation(sim_cfg, args_cli):
        from build_rsarm_usd import JOINTS, PARTS

        csv = static_check()
        print(f"== simulation ({args_cli.engine})")
        sim = SimulationContext(sim_cfg)
        scene = instantiate(SceneCfg(num_envs=1, env_spacing=2.0))
        robot = scene["robot"]
        sim.reset()
        print(f"  joints: {robot.joint_names}  bodies: {robot.body_names}")
        origin = scene.env_origins[0].cpu().numpy()

        def run(target: list[float], seconds: float):
            t = torch.tensor([target], device=sim.device)
            for _ in range(int(seconds / dt)):
                robot.actuators.target_command.set_position_index(value=t)
                robot.write_data_to_sim()
                sim.step(render=False)
                robot.update(dt)
            pos = robot.data.body_link_pos_w.torch[0].cpu().numpy() - origin
            quat = robot.data.body_link_quat_w.torch[0].cpu().numpy()
            return robot.data.joint_pos.torch[0].cpu().numpy(), pos, quat

        init = robot.data.default_joint_pos.torch[0].cpu().numpy()
        # the USD starts at q = 0; init_state is only the default, so write it before stepping
        robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch.clone())
        robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch.clone())
        robot.reset()
        jpos, pos, quat = run(init.tolist(), 2.0)
        check("hold initial pose", np.abs(jpos - init).max() < 0.05,
              f"joint pos {np.round(jpos, 4).tolist()} rad (target {np.round(init, 4).tolist()})")
        for name, _, _ in PARTS:  # every joint rotates about its child origin, so the origins stay put
            i = robot.body_names.index(name)
            err = np.linalg.norm(pos[i] - csv[name]["origin"]) * 1e3
            check(f"{name} origin at initial pose", err < 2.0, f"error {err:.2f} mm")

        for k, (jname, _, child, axis) in enumerate(JOINTS):
            target = init.tolist()
            target[robot.joint_names.index(jname)] += 0.5
            jpos, pos, quat = run(target, 2.0)
            i = robot.body_names.index(child)
            rotvec = Rotation.from_quat(quat[i]).as_rotvec()
            q = jpos[robot.joint_names.index(jname)]
            ok = np.allclose(rotvec, AXES[axis] * q, atol=0.02) and np.linalg.norm(pos[i] - csv[child]["origin"]) < 2e-3
            check(f"{jname} -> {child}", ok, f"q={q:.3f}  rotvec={np.round(rotvec, 3).tolist()}  "
                  f"origin shift {np.linalg.norm(pos[i] - csv[child]['origin']) * 1e3:.2f} mm")
            run(init.tolist(), 1.5)
    print(f"== {'ALL OK' if not fails else 'NG: ' + ', '.join(fails)}")


if __name__ == "__main__":
    main()
