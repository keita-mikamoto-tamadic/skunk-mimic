"""Play a wheel position-target excitation on the flywheel with its base fixed and record the response.

The sim stand-in for the real recording: the base fixed to the world (flywheel_fixed.usd), the motor driver's PD (kp, kd) as an explicit actuator at the physics step,
with the wheel's armature and friction set to known values, so the identification can be checked against them.

The physics step is the driver's position loop period (10 kHz, 0.1 ms). It matters: with the PD at 2 kHz the response
to sysid_excite.py differs from the 10 kHz one by 1.5 mrad rms, four times what 5 % more armature changes, and a 2 kHz
model fitted to 10 kHz data overestimates the armature by 10 % or more. The identification's own rollouts need the same
step.

Writes Isaac Sim SysID's compact CSV (t, q1, dq1, cmd1) at ``--record_hz``.

    isaaclab -p /workspace/isaaclab_ws/src/flywheel_sim/scripts/sysid_record.py --excite excite.csv \
        --out record.csv [--armature 2e-4] [--friction 0.01] [--viscous 0.0]
"""

import argparse

import numpy as np
import torch

from isaaclab.app import add_launcher_args, launch_simulation

from sim_base.physics import ENGINES, physics_cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--excite", required=True, help="CSV of t [s], q_cmd [rad] (sysid_excite.py).")
    parser.add_argument("--out", default="record.csv")
    parser.add_argument("--engine", choices=ENGINES, default="newton_mjwarp", help="Physics backend.")
    parser.add_argument("--dt", type=float, default=0.0001, help="Physics step = driver position loop period [s].")
    parser.add_argument("--record_hz", type=float, default=1000.0, help="Recording rate [Hz].")
    parser.add_argument("--kp", type=float, default=30.0, help="Driver position gain [N m/rad].")
    parser.add_argument("--kd", type=float, default=0.2, help="Driver velocity gain [N m s/rad].")
    parser.add_argument("--armature", type=float, default=0.0, help="True wheel armature [kg m^2].")
    parser.add_argument("--friction", type=float, default=0.0, help="True Coulomb friction [N m] (Newton).")
    parser.add_argument("--viscous", type=float, default=0.0, help="True viscous friction [N m s/rad] (Newton).")
    add_launcher_args(parser)
    args = parser.parse_args()

    excite = np.loadtxt(args.excite, delimiter=",", skiprows=1)
    cmd_t, cmd_q = excite[:, 0], excite[:, 1]
    record_every = round(1.0 / (args.record_hz * args.dt))

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import DCMotorCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.sim import SimulationContext
    from isaaclab.utils import instantiate, replace

    from flywheel_sim.robots import FLYWHEEL_CFG, FLYWHEEL_DIR, WHEEL_NO_LOAD_SPEED, WHEEL_STALL_TORQUE

    sim_cfg = sim_utils.SimulationCfg(device=args.device, dt=args.dt, physics=physics_cfg(args.engine, args.dt))
    with launch_simulation(sim_cfg, args):
        sim = SimulationContext(sim_cfg)
        robot_cfg = replace(
            FLYWHEEL_CFG,
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=replace(FLYWHEEL_CFG.spawn, usd_path=f"{FLYWHEEL_DIR}/flywheel_fixed.usd"),
            actuators={
                "wheel": DCMotorCfg(
                    joint_names_expr=["wheel"],
                    stiffness=args.kp,
                    damping=args.kd,
                    saturation_effort=WHEEL_STALL_TORQUE,
                    actuator_effort_limit=WHEEL_STALL_TORQUE,
                    actuator_velocity_limit=WHEEL_NO_LOAD_SPEED,
                )
            },
        )
        scene_cfg = InteractiveSceneCfg(num_envs=1, env_spacing=1.0)
        scene_cfg.robot = robot_cfg
        scene = instantiate(scene_cfg)
        robot = scene["robot"]
        sim.reset()
        robot.write_joint_armature_to_sim_index(armature=args.armature)
        robot.write_joint_friction_coefficient_to_sim_index(
            joint_friction_coeff=args.friction, joint_viscous_friction_coeff=args.viscous
        )
        dt = sim.get_physics_dt()

        rows = []
        for step in range(round(cmd_t[-1] / dt) + 1):
            t = step * dt
            q_cmd = float(cmd_q[np.searchsorted(cmd_t, t + 1e-9, side="right") - 1])
            if step % record_every == 0:
                rows.append((t, robot.data.joint_pos.torch[0, 0].item(), robot.data.joint_vel.torch[0, 0].item(), q_cmd))
            robot.actuators.target_command.set_position_index(value=torch.tensor([[q_cmd]], device=sim.device))
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(dt)

        np.savetxt(args.out, np.array(rows), delimiter=",", header="t,q1,dq1,cmd1", comments="", fmt="%.7g")
        print(f"wrote {args.out}: {len(rows)} samples")


if __name__ == "__main__":
    main()
