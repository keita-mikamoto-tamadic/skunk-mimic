"""Main optimization script for wheel system identification.

Loads recorded data, builds residual function, runs Levenberg-Marquardt
optimization via mujoco.sysid.optimize().

Usage:
    uv run sysid/optimize.py [--data <npz_file>] [--optimizer scipy]
"""

import argparse
import glob
import json
import os
import sys

import numpy as np
from mujoco import sysid

sys.path.insert(0, os.path.dirname(__file__))
from convert_recording import build_model_sequences
from define_params import build_params

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "sysid", "data")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "sysid", "results")


def find_latest_npz() -> str:
    """Find the latest .npz file in sysid/data/."""
    files = glob.glob(os.path.join(DATA_DIR, "*.npz"))
    if not files:
        raise FileNotFoundError(f"No .npz files found in {DATA_DIR}")
    return max(files, key=os.path.getmtime)


def save_results(params: sysid.ParameterDict, opt_result, output_dir: str):
    """Save optimization results as JSON."""
    os.makedirs(output_dir, exist_ok=True)

    result = {
        "parameters": {},
        "cost": float(opt_result.cost) if hasattr(opt_result, 'cost') else None,
        "success": bool(opt_result.success) if hasattr(opt_result, 'success') else None,
        "message": str(opt_result.message) if hasattr(opt_result, 'message') else None,
    }
    for name, param in params.items():
        val = param.value
        result["parameters"][name] = {
            "value": float(val) if np.isscalar(val) else float(np.asarray(val).flat[0]),
            "nominal": float(param.nominal) if np.isscalar(param.nominal) else float(np.asarray(param.nominal).flat[0]),
        }

    json_path = os.path.join(output_dir, "sysid_result.json")
    with open(json_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"Results saved to {json_path}")


def main():
    parser = argparse.ArgumentParser(description="Wheel system identification optimizer")
    parser.add_argument("--data", help="Path to .npz data file (default: latest)")
    parser.add_argument("--optimizer", default="scipy",
                        choices=["scipy", "mujoco"],
                        help="Optimizer backend (default: scipy)")
    args = parser.parse_args()

    # Load data
    data_path = args.data or find_latest_npz()
    print(f"Using data: {data_path}")

    ms = build_model_sequences(data_path)
    print(f"ModelSequences: {ms.name}")

    # Build parameters
    params = build_params()
    print("\nInitial parameters:")
    for name, p in params.items():
        print(f"  {name}: {p.value} (range: [{p.min_value}, {p.max_value}])")

    # Build residual function
    residual_fn = sysid.build_residual_fn(
        models_sequences=[ms],
        params=params,
    )

    # Optimize
    print(f"\nStarting optimization (optimizer: {args.optimizer})...")
    opt_params, opt_result = sysid.optimize(
        initial_params=params,
        residual_fn=residual_fn,
        optimizer=args.optimizer,
        verbose=True,
    )

    # Print results
    print("\nOptimized parameters:")
    for name, p in opt_params.items():
        nominal = np.asarray(p.nominal).flat[0]
        value = np.asarray(p.value).flat[0]
        print(f"  {name}: {nominal} -> {value}")

    # Save
    save_results(opt_params, opt_result, RESULTS_DIR)

    # Print MJCF update suggestions
    print("\n--- Suggested MJCF updates for sim/mimic_v2.xml ---")
    armature = float(np.asarray(opt_params["wheel_armature"].value).flat[0])
    damping = float(np.asarray(opt_params["wheel_damping"].value).flat[0])
    frictionloss = float(np.asarray(opt_params["wheel_frictionloss"].value).flat[0])
    ground_fric = float(np.asarray(opt_params["ground_friction"].value).flat[0])
    print(f'  wheel joints: armature="{armature:.6f}" damping="{damping:.6f}" frictionloss="{frictionloss:.6f}"')
    print(f'  ground geom:  friction="{ground_fric:.4f} 0.005 0.0001"')


if __name__ == "__main__":
    main()
