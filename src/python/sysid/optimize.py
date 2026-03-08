"""Main optimization script for wheel system identification.

Uses scipy.optimize.least_squares with a custom rollout that handles
the runtime actuator mode switching (STOP/POSITION/VELOCITY).

Usage:
    uv run sysid/optimize.py [--data <npz_file>]
"""

import argparse
import glob
import json
import os
import sys

import mujoco
import numpy as np
from mujoco import sysid
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(__file__))
from convert_recording import load_npz, trim_to_excitation, MODEL_PATH, ACTUATOR_NAMES
from define_params import build_params

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "sysid", "data")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "sysid", "results")

BASE_KP = 140.0 / (2.0 * 3.141592653589793)   # moteus kp=140 Nm/rev → Nm/rad
BASE_KV = 0.5 / (2.0 * 3.141592653589793)     # moteus kd=0.5 Nm/(rev/s) → Nm/(rad/s)
SUBSTEPS = 3

# Wheel axis indices for residual
WHEEL_R = 2
WHEEL_L = 5


def find_latest_npz() -> str:
    files = glob.glob(os.path.join(DATA_DIR, "*.npz"))
    if not files:
        raise FileNotFoundError(f"No .npz files found in {DATA_DIR}")
    return max(files, key=os.path.getmtime)


def run_sim_rollout(data: dict, params: sysid.ParameterDict) -> dict:
    """Run sim rollout with parameter modifiers applied, handling mode switching."""
    spec = mujoco.MjSpec.from_file(MODEL_PATH)
    spec = sysid.apply_param_modifiers_spec(params, spec)
    model = spec.compile()
    mj_data = mujoco.MjData(model)

    # Initial state from recording
    mj_data.qpos[:] = data["status_position"][0]
    mj_data.qvel[:] = data["status_velocity"][0]
    mujoco.mj_forward(model, mj_data)

    cmd_times = data["cmd_timestamps"]
    cmd_refs = data["cmd_ref_val"]
    cmd_states = data["cmd_motor_state"]
    cmd_kp = data.get("cmd_kp_scale", np.ones_like(cmd_refs))
    cmd_kv = data.get("cmd_kv_scale", np.ones_like(cmd_refs))

    act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
               for n in ACTUATOR_NAMES]

    dt = model.opt.timestep
    out_times, out_pos, out_vel = [], [], []
    cmd_idx = 0
    sim_time = 0.0
    max_time = cmd_times[-1] + 0.01

    while sim_time < max_time:
        while cmd_idx + 1 < len(cmd_times) and cmd_times[cmd_idx + 1] <= sim_time:
            cmd_idx += 1

        for i in range(6):
            aid = act_ids[i]
            motor_state = cmd_states[cmd_idx, i]
            ref_val = cmd_refs[cmd_idx, i]
            kp_s = cmd_kp[cmd_idx, i] if cmd_kp[cmd_idx, i] > 0 else 1.0
            kv_s = cmd_kv[cmd_idx, i] if cmd_kv[cmd_idx, i] > 0 else 1.0

            if motor_state == 2:  # POSITION
                kp = BASE_KP * kp_s
                kv = BASE_KV * kv_s
                model.actuator_gainprm[aid, 0] = kp
                model.actuator_biasprm[aid, :3] = [0.0, -kp, -kv]
                mj_data.ctrl[aid] = ref_val
            elif motor_state == 3:  # VELOCITY
                kv = BASE_KV * kv_s
                model.actuator_gainprm[aid, 0] = kv
                model.actuator_biasprm[aid, :3] = [0.0, 0.0, -kv]
                mj_data.ctrl[aid] = ref_val
            elif motor_state == 1:  # STOP
                kp = BASE_KP
                kv = BASE_KV
                model.actuator_gainprm[aid, 0] = kp
                model.actuator_biasprm[aid, :3] = [0.0, -kp, -kv]
                mj_data.ctrl[aid] = mj_data.qpos[i]  # hold current
            else:  # OFF
                model.actuator_gainprm[aid, 0] = 0.0
                model.actuator_biasprm[aid, :3] = [0.0, 0.0, 0.0]
                mj_data.ctrl[aid] = 0.0

        for _ in range(SUBSTEPS):
            mujoco.mj_step(model, mj_data)

        sim_time += dt * SUBSTEPS
        out_times.append(sim_time)
        out_pos.append(mj_data.qpos[:6].copy())
        out_vel.append(mj_data.qvel[:6].copy())

    return {
        "times": np.array(out_times),
        "position": np.array(out_pos),
        "velocity": np.array(out_vel),
    }


def compute_residual(x: np.ndarray, params: sysid.ParameterDict,
                     data: dict) -> np.ndarray:
    """Compute residual: sim prediction - real measurement for wheel axes."""
    params.update_from_vector(x)
    sim = run_sim_rollout(data, params)

    # Interpolate sim onto real timestamps
    real_t = data["status_timestamps"]
    residuals = []
    for ax_idx in [WHEEL_R, WHEEL_L]:
        # Position residual
        sim_pos = np.interp(real_t, sim["times"], sim["position"][:, ax_idx])
        real_pos = data["status_position"][:, ax_idx]
        residuals.append(sim_pos - real_pos)

        # Velocity residual
        sim_vel = np.interp(real_t, sim["times"], sim["velocity"][:, ax_idx])
        real_vel = data["status_velocity"][:, ax_idx]
        residuals.append(sim_vel - real_vel)

    return np.concatenate(residuals)


def save_results(params: sysid.ParameterDict, opt_result, output_dir: str):
    os.makedirs(output_dir, exist_ok=True)
    result = {
        "parameters": {},
        "cost": float(opt_result.cost),
        "success": bool(opt_result.success),
        "message": str(opt_result.message),
    }
    for name, param in params.items():
        val = float(np.asarray(param.value).flat[0])
        nom = float(np.asarray(param.nominal).flat[0])
        result["parameters"][name] = {"value": val, "nominal": nom}

    json_path = os.path.join(output_dir, "sysid_result.json")
    with open(json_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"Results saved to {json_path}")


def main():
    parser = argparse.ArgumentParser(description="Wheel system identification optimizer")
    parser.add_argument("--data", help="Path to .npz data file (default: latest)")
    args = parser.parse_args()

    data_path = args.data or find_latest_npz()
    print(f"Using data: {data_path}")

    data = load_npz(data_path)
    data = trim_to_excitation(data)

    params = build_params()
    x0 = params.as_vector()
    lb, ub = params.get_bounds()

    print("\nInitial parameters:")
    for name, p in params.items():
        print(f"  {name}: {np.asarray(p.value).flat[0]}")

    # Quick sensitivity check
    r0 = compute_residual(x0, params, data)
    print(f"\nInitial cost: {np.sum(r0**2):.4f} (residual len: {len(r0)})")

    print("\nStarting optimization...")
    result = least_squares(
        compute_residual,
        x0,
        args=(params, data),
        bounds=(lb, ub),
        method="trf",
        verbose=2,
    )

    params.update_from_vector(result.x)
    print("\nOptimized parameters:")
    for name, p in params.items():
        nom = float(np.asarray(p.nominal).flat[0])
        val = float(np.asarray(p.value).flat[0])
        print(f"  {name}: {nom} -> {val}")

    print(f"Final cost: {result.cost:.4f}")
    save_results(params, result, RESULTS_DIR)

    # MJCF suggestions
    armature = float(np.asarray(params["wheel_armature"].value).flat[0])
    damping = float(np.asarray(params["wheel_damping"].value).flat[0])
    frictionloss = float(np.asarray(params["wheel_frictionloss"].value).flat[0])
    ground_fric = float(np.asarray(params["ground_friction"].value).flat[0])
    print(f'\n--- Suggested MJCF updates for sim/mimic_v2.xml ---')
    print(f'  wheel joints: armature="{armature:.6f}" damping="{damping:.6f}" frictionloss="{frictionloss:.6f}"')
    print(f'  ground geom:  friction="{ground_fric:.4f} 0.005 0.0001"')


if __name__ == "__main__":
    main()
