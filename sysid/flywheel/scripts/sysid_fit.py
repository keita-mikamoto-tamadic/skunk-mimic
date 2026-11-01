"""flywheel ホイール軸の armature/摩擦を実測データに mujoco.sysid でフィットする (ホスト実行)。

flywheel_sysid dora ノードの記録 CSV (sysid/flywheel/data/record_*.csv, t,q1,dq1,cmd1 @1kHz)
に対し、robot_file/fly_wheel/flywheel_sysid.xml (ベース固定 + PD アクチュエータ) の
gain_scale / tau_max / armature / damping / frictionloss をロールアウト残差最小化で推定する。
依存は mujoco + numpy + matplotlib のみ (isaaclab 不要)。src/python の uv 環境で回す:

    cd src/python && uv run ../../sysid/flywheel/scripts/sysid_fit.py \
        ../../sysid/flywheel/data/record_*.csv

結果: fit_result.json と、記録ごとの実測 vs ロールアウト比較 fit_<record>.jpg
(いずれも最初の record と同じディレクトリ)。--model / --out / --no-plot で変更可。
同定の健全性チェックはシム既知真値で可能: sysid_record.py --armature 2e-4 --friction 0.01
で作った CSV をフィットして真値が戻るか見る。
"""

import argparse
import json
import os

import mujoco
import numpy as np
from mujoco import sysid

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# <repo>/sysid/flywheel/scripts/ → 3 階層上がリポジトリルート
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))


def find_default_model() -> str:
    path = os.path.join(REPO_ROOT, "robot_file", "fly_wheel", "flywheel_sysid.xml")
    if os.path.exists(path):
        return path
    raise FileNotFoundError("flywheel_sysid.xml not found (pass --model)")


MJCF_KP = 30.0  # flywheel_sysid.xml の <position kp>
MJCF_KV = 0.2   # 同 kv


def _joint_modifier(attr: str):
    def modifier(spec: mujoco.MjSpec, param: sysid.Parameter):
        for j in spec.joints:
            if j.name == "wheel":
                setattr(j, attr, float(np.asarray(param.value).flat[0]))
                break
        return spec
    return modifier


def _gain_scale_modifier():
    """kp/kv を同率スケール (firmware の imp_kp/imp_kd 比は既知として保存)。

    実測の Kd/Kp 比 ≈ 1/144 は firmware 設定比 0.2/30 = 1/150 とほぼ一致しており、
    単位換算 (A/rad → N·m/rad、おそらく torq_const) が両ゲインに同率で掛かる
    という仮説に基づく。<position> は内部的に general actuator
    (gainprm[0]=kp, biasprm=[0,-kp,-kv]) なので 3 箇所を書き換える。
    """
    def modifier(spec: mujoco.MjSpec, param: sysid.Parameter):
        s = float(np.asarray(param.value).flat[0])
        for a in spec.actuators:
            if a.name == "wheel":
                a.gainprm[0] = MJCF_KP * s
                a.biasprm[1] = -MJCF_KP * s
                a.biasprm[2] = -MJCF_KV * s
                break
        return spec
    return modifier


def _tau_max_modifier():
    """アクチュエータ出力トルクの飽和 (forcerange)。"""
    def modifier(spec: mujoco.MjSpec, param: sysid.Parameter):
        tm = float(np.asarray(param.value).flat[0])
        for a in spec.actuators:
            if a.name == "wheel":
                a.forcerange = [-tm, tm]
                break
        return spec
    return modifier


def build_params() -> sysid.ParameterDict:
    return sysid.ParameterDict({
        # 実効 kp = 30*s [N·m/rad]。nominal 0.085 = torq_const (imp_kp が A/rad 仮説)
        "gain_scale": sysid.Parameter(
            name="gain_scale", nominal=0.085, min_value=0.01, max_value=1.0,
            modifier=_gain_scale_modifier(),
        ),
        "tau_max": sysid.Parameter(
            name="tau_max", nominal=0.4, min_value=0.05, max_value=5.0,
            modifier=_tau_max_modifier(),
        ),
        "armature": sysid.Parameter(
            name="armature", nominal=2e-4, min_value=1e-6, max_value=5e-3,
            modifier=_joint_modifier("armature"),
        ),
        "damping": sysid.Parameter(
            name="damping", nominal=1e-3, min_value=0.0, max_value=0.05,
            modifier=_joint_modifier("damping"),
        ),
        "frictionloss": sysid.Parameter(
            name="frictionloss", nominal=1e-2, min_value=0.0, max_value=0.5,
            modifier=_joint_modifier("frictionloss"),
        ),
    })


def rollout_fitted(spec: mujoco.MjSpec, csv_path: str) -> tuple:
    """フィット済み spec で記録と同じ入力をロールアウトし (t, q, dq, cmd, q_sim, dq_sim) を返す。"""
    d = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    t, q, dq, cmd = d.T
    m = spec.compile()
    data = mujoco.MjData(m)
    data.qpos[0], data.qvel[0] = q[0], dq[0]
    sub = round((t[1] - t[0]) / m.opt.timestep)
    q_sim = np.empty(len(t))
    dq_sim = np.empty(len(t))
    for k in range(len(t)):
        q_sim[k], dq_sim[k] = data.qpos[0], data.qvel[0]
        data.ctrl[0] = cmd[k]
        for _ in range(sub):
            mujoco.mj_step(m, data)
    return t, q, dq, cmd, q_sim, dq_sim


def save_plot(csv_path: str, rollout, out_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t, q, dq, cmd, q_sim, dq_sim = rollout
    err = q_sim - q
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(t, cmd, "k--", lw=0.8, label="cmd")
    axes[0].plot(t, q, lw=0.8, label="real q")
    axes[0].plot(t, q_sim, lw=0.8, label="fit q")
    axes[0].set_ylabel("q [rad]")
    axes[0].legend(loc="upper right", ncol=3)
    axes[1].plot(t, dq, lw=0.8, label="real dq")
    axes[1].plot(t, dq_sim, lw=0.8, label="fit dq")
    axes[1].set_ylabel("dq [rad/s]")
    axes[1].legend(loc="upper right", ncol=2)
    axes[2].plot(t, err * 1e3, lw=0.8, color="tab:red")
    axes[2].set_ylabel("q error [mrad]")
    axes[2].set_xlabel("t [s]")
    fig.suptitle(f"{os.path.basename(csv_path)}  —  fit rms {err.std()*1e3:.1f} mrad, "
                 f"max {np.abs(err).max()*1e3:.1f} mrad")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"wrote {out_path}  (rms {err.std()*1e3:.1f} mrad)")


def build_model_sequences(csv_path: str, model_path: str) -> sysid.ModelSequences:
    d = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    t, q, dq, cmd = d.T
    print(f"{os.path.basename(csv_path)}: {len(t)} samples, {t[-1]:.1f} s, "
          f"|q|max {np.abs(q).max():.3f} rad")

    spec = mujoco.MjSpec.from_file(model_path)
    model = spec.compile()

    control = sysid.TimeSeries.from_control_names(
        times=t, data=cmd[:, None], model=model, names=["wheel"])
    sensordata = sysid.TimeSeries.from_names(
        times=t, data=np.column_stack([q, dq]), model=model,
        names=["jpos_wheel", "jvel_wheel"])
    initial_state = sysid.create_initial_state(
        model=model, qpos=q[:1], qvel=dq[:1])

    name = os.path.splitext(os.path.basename(csv_path))[0]
    return sysid.ModelSequences(
        name=name, spec=spec, sequence_name=name,
        initial_state=initial_state, control=control, sensordata=sensordata)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("records", nargs="+", help="record_*.csv (t,q1,dq1,cmd1)")
    parser.add_argument("--model", default=None, help="SysID MJCF (default: auto-locate)")
    parser.add_argument("--optimizer", default="mujoco", choices=["scipy", "mujoco"])
    parser.add_argument("--out", default=None,
                        help="result JSON (default: fit_result.json next to the first record)")
    parser.add_argument("--no-plot", action="store_true",
                        help="フィット後の実測 vs ロールアウト比較 jpg を保存しない")
    args = parser.parse_args()

    model_path = args.model or find_default_model()
    print(f"model: {model_path}")

    sequences = [build_model_sequences(p, model_path) for p in args.records]
    params = build_params()

    residual_fn = sysid.build_residual_fn(
        models_sequences=sequences,
        sensor_weights={"jpos_wheel": 1.0, "jvel_wheel": 1.0},
    )

    print(f"optimizing ({args.optimizer}) ...")
    opt_params, opt_result = sysid.optimize(
        initial_params=params, residual_fn=residual_fn,
        optimizer=args.optimizer, verbose=True)

    print("\n=== fitted parameters ===")
    result = {"records": [os.path.basename(p) for p in args.records],
              "model": os.path.basename(model_path),
              "cost": float(getattr(opt_result, "cost", float("nan"))),
              "parameters": {}}
    for name, p in opt_params.items():
        val = float(np.asarray(p.value).flat[0])
        nom = float(np.asarray(p.nominal).flat[0])
        result["parameters"][name] = val
        print(f"  {name:14s} {nom:g} -> {val:.6g}")

    s = result["parameters"]["gain_scale"]
    result["kp_eff_Nm_per_rad"] = MJCF_KP * s
    result["kd_eff_Nms_per_rad"] = MJCF_KV * s

    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.records[0])),
                                   "fit_result.json")
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nwrote {out}")
    print(f"実効ゲイン: kp={MJCF_KP * s:.3f} N·m/rad, kd={MJCF_KV * s:.5f} N·m·s/rad "
          f"(MJCF 値 x {s:.4f}); tau_max={result['parameters']['tau_max']:.3f} N·m")
    print("MJCF: <joint name=\"wheel\" armature=\"{armature:.6g}\" damping=\"{damping:.6g}\" "
          "frictionloss=\"{frictionloss:.6g}\"/>".format(**result["parameters"]))

    if not args.no_plot:
        # フィット済みパラメータを適用した spec で各記録をロールアウトし比較図を保存
        spec = mujoco.MjSpec.from_file(model_path)
        for p in opt_params.values():
            spec = p.modifier(spec, p)
        for csv_path in args.records:
            name = os.path.splitext(os.path.basename(csv_path))[0]
            jpg = os.path.join(os.path.dirname(os.path.abspath(csv_path)), f"fit_{name}.jpg")
            save_plot(csv_path, rollout_fitted(spec, csv_path), jpg)


if __name__ == "__main__":
    main()
