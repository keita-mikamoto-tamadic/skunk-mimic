"""Validation script: compare sim rollout (before/after optimization) with real data.

Generates time-series comparison plots for wheel position and velocity.

Usage:
    uv run sysid/validate.py [--data <npz>] [--results <results_dir>]
"""

import argparse
import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from mujoco import sysid

sys.path.insert(0, os.path.dirname(__file__))
from convert_recording import load_npz, MODEL_PATH, ACTUATOR_NAMES
from define_params import build_params

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "sysid", "data")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "sysid", "results")

# Wheel axes for plotting
WHEEL_AXES = [2, 5]  # wheel_r, wheel_l
WHEEL_NAMES = ["wheel_r", "wheel_l"]


def find_latest_npz() -> str:
    files = glob.glob(os.path.join(DATA_DIR, "*.npz"))
    if not files:
        raise FileNotFoundError(f"No .npz files in {DATA_DIR}")
    return max(files, key=os.path.getmtime)


def load_opt_params(results_dir: str) -> sysid.ParameterDict | None:
    """Load optimized parameters from JSON results."""
    json_path = os.path.join(results_dir, "sysid_result.json")
    if not os.path.exists(json_path):
        print(f"No optimization results found at {json_path}")
        return None

    with open(json_path) as f:
        result = json.load(f)

    params = build_params()
    for name, info in result["parameters"].items():
        if name in params:
            params[name].update_from_vector([info["value"]])
    return params


def run_sim_rollout(data: dict,
                    params: sysid.ParameterDict | None = None) -> dict:
    """Run a simulation rollout and return predicted sensor data."""
    spec = mujoco.MjSpec.from_file(MODEL_PATH)
    if params is not None:
        spec = sysid.apply_param_modifiers_spec(params, spec)

    model = spec.compile()
    mj_data = mujoco.MjData(model)

    # Set initial state from recording
    qpos0 = data["status_position"][0]
    qvel0 = data["status_velocity"][0]
    for i in range(min(len(qpos0), model.nq)):
        mj_data.qpos[i] = qpos0[i]
    for i in range(min(len(qvel0), model.nv)):
        mj_data.qvel[i] = qvel0[i]
    mujoco.mj_forward(model, mj_data)

    # Replay commands
    cmd_times = data["cmd_timestamps"]
    cmd_refs = data["cmd_ref_val"]
    cmd_states = data["cmd_motor_state"]
    cmd_kp = data.get("cmd_kp_scale", np.ones_like(cmd_refs))
    cmd_kv = data.get("cmd_kv_scale", np.ones_like(cmd_refs))

    BASE_KP = 50.0
    BASE_KV = 20.0

    act_ids = []
    for name in ACTUATOR_NAMES:
        act_ids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))

    dt = model.opt.timestep
    substeps = 3

    out_times, out_pos, out_vel = [], [], []
    cmd_idx = 0
    sim_time = 0.0
    max_time = cmd_times[-1] + 0.1

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
            else:
                model.actuator_gainprm[aid, 0] = 0.0
                model.actuator_biasprm[aid, :3] = [0.0, 0.0, 0.0]
                mj_data.ctrl[aid] = 0.0

        for _ in range(substeps):
            mujoco.mj_step(model, mj_data)

        sim_time += dt * substeps
        out_times.append(sim_time)
        out_pos.append(mj_data.qpos[:6].copy())
        out_vel.append(mj_data.qvel[:6].copy())

    return {
        "times": np.array(out_times),
        "position": np.array(out_pos),
        "velocity": np.array(out_vel),
    }


def plot_comparison(real_data: dict,
                    sim_nominal: dict, sim_optimized: dict | None,
                    output_dir: str):
    """Plot real vs sim time series for wheel axes."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)

    real_t = real_data["status_timestamps"]

    for col, (ax_idx, ax_name) in enumerate(zip(WHEEL_AXES, WHEEL_NAMES)):
        ax = axes[0, col]
        ax.plot(real_t, real_data["status_position"][:, ax_idx],
                label="Real", color="black", linewidth=1.5)
        ax.plot(sim_nominal["times"], sim_nominal["position"][:, ax_idx],
                label="Sim (nominal)", color="blue", alpha=0.7)
        if sim_optimized is not None:
            ax.plot(sim_optimized["times"], sim_optimized["position"][:, ax_idx],
                    label="Sim (optimized)", color="red", alpha=0.7)
        ax.set_ylabel("Position [rad]")
        ax.set_title(f"{ax_name} position")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[1, col]
        ax.plot(real_t, real_data["status_velocity"][:, ax_idx],
                label="Real", color="black", linewidth=1.5)
        ax.plot(sim_nominal["times"], sim_nominal["velocity"][:, ax_idx],
                label="Sim (nominal)", color="blue", alpha=0.7)
        if sim_optimized is not None:
            ax.plot(sim_optimized["times"], sim_optimized["velocity"][:, ax_idx],
                    label="Sim (optimized)", color="red", alpha=0.7)
        ax.set_ylabel("Velocity [rad/s]")
        ax.set_xlabel("Time [s]")
        ax.set_title(f"{ax_name} velocity")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    fig.suptitle("System Identification: Real vs Sim", fontsize=14)
    fig.tight_layout()

    os.makedirs(output_dir, exist_ok=True)
    plot_path = os.path.join(output_dir, "sysid_validation.png")
    fig.savefig(plot_path, dpi=150)
    print(f"Plot saved to {plot_path}")


def main():
    parser = argparse.ArgumentParser(description="Validate sysid results")
    parser.add_argument("--data", help="Path to .npz recording (default: latest)")
    parser.add_argument("--results", default=RESULTS_DIR,
                        help="Path to optimization results directory")
    args = parser.parse_args()

    npz_path = args.data or find_latest_npz()
    print(f"Loading real data: {npz_path}")
    real_data = load_npz(npz_path)

    print("Running nominal sim rollout...")
    sim_nominal = run_sim_rollout(real_data)

    opt_params = load_opt_params(args.results)
    sim_optimized = None
    if opt_params is not None:
        print("Running optimized sim rollout...")
        sim_optimized = run_sim_rollout(real_data, opt_params)

    plot_comparison(real_data, sim_nominal, sim_optimized, args.results)


if __name__ == "__main__":
    main()
